//! ENS160 Air Quality Sensor for RP Pico W
//!
//! Reads raw air quality data from an ENS160 sensor over I2C and logs it.
//! Supports both RTT (with debug probe) and USB serial logging.

#![no_std]
#![no_main]

use cyw43::aligned_bytes;
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Async, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use ens160_aq::data::InterruptPinConfig;
use ens160_aq::Ens160;
use static_cell::StaticCell;

use defmt::unwrap;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

/// I2C bus type alias
type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, Async>>;

/// ENS160 device type alias
type Ens160Device = Ens160<I2cDevice<'static, NoopRawMutex, I2c<'static, I2C0, Async>>, Delay>;

/// Warmup time for ENS160 sensor in seconds
const WARMUP_TIME: u64 = 180;

/// Read interval in seconds
const READ_INTERVAL: u64 = 5;

/// Scan I2C bus for devices and log results
async fn scan_i2c_bus(i2c: &mut I2c<'static, I2C0, Async>) {
    use embedded_hal_async::i2c::I2c as _;

    log::info!("Scanning I2C bus...");
    defmt::info!("Scanning I2C bus...");

    let mut found_count = 0;

    // Scan valid 7-bit addresses (0x08-0x77, excluding reserved ranges)
    for addr in 0x08u8..=0x77u8 {
        // Try to read a single byte from each address
        let mut buf = [0u8; 1];
        if i2c.read(addr, &mut buf).await.is_ok() {
            log::info!("  Found device at address 0x{:02X}", addr);
            defmt::info!("  Found device at address 0x{:02X}", addr);
            found_count += 1;
        }
    }

    if found_count == 0 {
        log::warn!("No I2C devices found!");
        defmt::warn!("No I2C devices found!");
    } else {
        log::info!("Found {} device(s)", found_count);
        defmt::info!("Found {} device(s)", found_count);
    }
}

/// Struct to hold ENS160 sensor readings
struct Ens160Readings {
    eco2: u16,
    etoh: u16,
    tvoc: u16,
    aqi: u8,
}

/// Initialize the ENS160 sensor
async fn initialize_ens160(
    i2c_device: I2cDevice<'static, NoopRawMutex, I2c<'static, I2C0, Async>>,
) -> Option<Ens160Device> {
    // Use secondary address (0x53) for SparkFun board with ADDR jumper set high
    // Use Ens160::new() instead if your board uses address 0x52
    let mut ens160 = Ens160::new_secondary_address(i2c_device, Delay);

    if let Err(e) = ens160.initialize().await {
        log::error!("Failed to initialize ENS160: {:?}", e);
        defmt::error!("Failed to initialize ENS160: {}", Debug2Format(&e));
        return None;
    }
    log::info!("ENS160 initialized successfully");
    defmt::info!("ENS160 initialized successfully");

    // Configure interrupt pin for new data notifications
    match ens160
        .config_interrupt_pin(
            InterruptPinConfig::builder()
                .push_pull()
                .on_new_data()
                .enable_interrupt()
                .build(),
        )
        .await
    {
        Ok(val) => {
            log::info!("ENS160 interrupt pin configured: {}", val);
            defmt::info!("ENS160 interrupt pin configured: {}", val);
        }
        Err(e) => {
            log::error!("Failed to configure ENS160 interrupt pin: {:?}", e);
            defmt::error!("Failed to configure ENS160 interrupt pin: {}", Debug2Format(&e));
            return None;
        }
    }

    Some(ens160)
}

/// Read raw data from ENS160 sensor (single reading, no median)
async fn read_ens160(
    ens160: &mut Ens160Device,
    int_pin: &mut Input<'static>,
) -> Result<Ens160Readings, &'static str> {
    // Wait for interrupt indicating new data is ready
    int_pin.wait_for_low().await;
    defmt::debug!("ENS160 interrupt received - data ready");

    let status = ens160
        .get_status()
        .await
        .map_err(|_| "Failed to get ENS160 status")?;
    defmt::debug!("ENS160 status: {}", Debug2Format(&status));

    let eco2 = ens160
        .get_eco2()
        .await
        .map_err(|_| "Failed to get eCO2")?;

    let etoh = ens160
        .get_etoh()
        .await
        .map_err(|_| "Failed to get ethanol")?;

    let tvoc = ens160
        .get_tvoc()
        .await
        .map_err(|_| "Failed to get VOCs")?;

    let aqi = ens160
        .get_airquality_index()
        .await
        .map_err(|_| "Failed to get Air Quality Index")?;

    let readings = Ens160Readings {
        eco2: eco2.get_value(),
        etoh,
        tvoc,
        aqi: aqi as u8,
    };

    // Log to both USB serial and RTT
    log::info!(
        "AQI: {}, eCO2: {} ppm, Ethanol: {} ppb, VOCs: {} ppb",
        readings.aqi, readings.eco2, readings.etoh, readings.tvoc
    );
    defmt::info!(
        "ENS160 Raw Readings - AQI: {}, eCO2: {} ppm, Ethanol: {} ppb, VOCs: {} ppb",
        readings.aqi, readings.eco2, readings.etoh, readings.tvoc
    );

    Ok(readings)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize USB serial logger
    let driver = Driver::new(p.USB, Irqs);
    spawner.spawn(unwrap!(logger_task(driver)));

    // Give USB time to enumerate before sending logs
    Timer::after(Duration::from_secs(2)).await;
    log::info!("ENS160 Air Quality Sensor starting...");

    // Initialize CYW43 (WiFi chip) for onboard LED
    let fw = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Initialize I2C bus for ENS160 sensor
    // Using GPIO4 (SDA) and GPIO5 (SCL) - adjust pins as needed for your wiring
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, I2cConfig::default());

    // Wait for sensor to power up before scanning
    log::info!("Waiting for I2C devices to power up...");
    defmt::info!("Waiting for I2C devices to power up...");
    Timer::after(Duration::from_millis(2000)).await;

    // Scan I2C bus to find devices
    scan_i2c_bus(&mut i2c).await;

    // Create shared I2C bus
    static I2C_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Create I2C device for ENS160
    let ens160_i2c = I2cDevice::new(i2c_bus);

    // ENS160 interrupt pin - adjust as needed for your wiring
    let mut ens160_int = Input::new(p.PIN_6, Pull::Up);

    // Initialize ENS160
    let mut ens160 = match initialize_ens160(ens160_i2c).await {
        Some(sensor) => sensor,
        None => {
            log::error!("Failed to initialize ENS160 - halting");
            defmt::error!("Failed to initialize ENS160 - halting");
            loop {
                // Blink LED rapidly to indicate error
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Wait for ENS160 warmup period
    log::info!("Waiting for ENS160 warmup period of {} seconds", WARMUP_TIME);
    log::info!("LED will blink slowly during warmup...");
    defmt::info!("Waiting for ENS160 warmup period of {} seconds", WARMUP_TIME);

    for i in 0..WARMUP_TIME {
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(200)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(800)).await;

        if (i + 1) % 30 == 0 {
            log::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
            defmt::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
        }
    }

    log::info!("Warmup complete - starting continuous readings");
    defmt::info!("Warmup complete - starting continuous readings");

    // Main sensor reading loop
    loop {
        // Indicate reading with LED
        control.gpio_set(0, true).await;

        match read_ens160(&mut ens160, &mut ens160_int).await {
            Ok(_readings) => {
                defmt::debug!("Sensor read successful");
            }
            Err(e) => {
                log::error!("Sensor read failed: {}", e);
                defmt::error!("Sensor read failed: {}", e);
            }
        }

        control.gpio_set(0, false).await;

        // Wait before next reading
        Timer::after(Duration::from_secs(READ_INTERVAL)).await;
    }
}
