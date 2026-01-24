//! SCD40 CO2 Sensor with AHT20/BMP280 for RP Pico W
//!
//! Reads CO2 data from an SCD40 sensor, temperature/humidity from AHT20,
//! and pressure from BMP280. Submits sensor data to a REST API endpoint over WiFi.
//! Logs via USB serial (optional) and RTT (for debug probe).

#![no_std]
#![no_main]

extern crate alloc;

mod network;
mod sensors;
mod types;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use cyw43::JoinOptions;
use cyw43_firmware::{CYW43_43439A0 as FIRMWARE, CYW43_43439A0_CLM as CLM};
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::Config as NetConfig;
use embassy_net::StackResources;
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Async, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
#[cfg(feature = "usb-logging")]
use embassy_rp::peripherals::USB;
#[cfg(feature = "usb-logging")]
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
#[cfg(feature = "usb-logging")]
use embassy_usb_logger::ReceiverHandler;
use static_cell::StaticCell;

use crate::network::{check_network_status, submit_sensor_data};
use crate::sensors::{Aht20Sensor, Bmp280Sensor, Scd40Sensor};
use crate::types::{NetworkStatus, SensorPayload, SensorReadings};

use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "usb-logging")]
use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;

#[cfg(feature = "usb-logging")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[cfg(not(feature = "usb-logging"))]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

// WiFi configuration loaded from .env file at build time
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Warmup time for SCD40 sensor in seconds (faster than ENS160's 180s)
const WARMUP_TIME: u64 = 60;

/// Read interval in seconds
const READ_INTERVAL: u64 = 120;

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

/// USB serial logger task (only compiled when usb-logging feature is enabled)
#[cfg(feature = "usb-logging")]
struct Handler;

#[cfg(feature = "usb-logging")]
impl ReceiverHandler for Handler {
    fn new() -> Self {
        Handler
    }

    async fn handle_data(&self, _data: &[u8]) {
        // Ignore incoming data from USB serial
    }
}

#[cfg(feature = "usb-logging")]
#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, Handler);
}

/// I2C bus type alias
type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, Async>>;

/// Scan I2C bus for devices and log results
async fn scan_i2c_bus(i2c: &mut I2c<'static, I2C0, Async>) {
    use embedded_hal_async::i2c::I2c as _;

    defmt::info!("Scanning I2C bus...");
    log::info!("Scanning I2C bus...");

    let mut found_count = 0;

    // Scan valid 7-bit addresses (0x08-0x77, excluding reserved ranges)
    for addr in 0x08u8..=0x77u8 {
        // Try to read a single byte from each address
        let mut buf = [0u8; 1];
        if i2c.read(addr, &mut buf).await.is_ok() {
            defmt::info!("  Found device at address 0x{:02X}", addr);
            log::info!("  Found device at address 0x{:02X}", addr);
            found_count += 1;
        }
    }

    if found_count == 0 {
        defmt::warn!("No I2C devices found!");
        log::warn!("No I2C devices found!");
    } else {
        defmt::info!("Found {} device(s)", found_count);
        log::info!("Found {} device(s)", found_count);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize heap for TLS RSA support (32KB)
    {
        use core::mem::MaybeUninit;

        const HEAP_SIZE: usize = 32 * 1024;
        static HEAP_MEM: StaticCell<[MaybeUninit<u8>; HEAP_SIZE]> = StaticCell::new();
        let heap_mem = HEAP_MEM.init([const { MaybeUninit::uninit() }; HEAP_SIZE]);
        unsafe { HEAP.init(heap_mem.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_rp::init(Default::default());

    // Initialize random number generator for network stack
    let mut rng = RoscRng;

    // Initialize USB serial logger (only when usb-logging feature is enabled)
    #[cfg(feature = "usb-logging")]
    {
        let usb_driver = Driver::new(p.USB, Irqs);
        spawner.spawn(logger_task(usb_driver)).unwrap();
        // Give USB time to enumerate before logging
        Timer::after(Duration::from_millis(100)).await;
    }

    defmt::info!("SCD40 CO2 Sensor with WiFi starting...");
    log::info!("SCD40 CO2 Sensor with WiFi starting...");

    // Initialize CYW43 (WiFi chip) for onboard LED and networking
    let fw = FIRMWARE;
    let clm = CLM;

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
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Initialize network stack
    let net_config = NetConfig::dhcpv4(Default::default());
    let seed = rng.next_u64();

    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        net_config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(net_task(net_runner)).unwrap();

    // Connect to WiFi
    defmt::info!("Connecting to WiFi network: {}", WIFI_SSID);
    log::info!("Connecting to WiFi network: {}", WIFI_SSID);

    loop {
        match control.join(WIFI_SSID, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
            Ok(()) => break,
            Err(e) => {
                defmt::warn!("WiFi join failed: {}", Debug2Format(&e));
                log::warn!("WiFi join failed, retrying...");
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    defmt::info!("WiFi connected, waiting for link...");
    log::info!("WiFi connected, waiting for link...");
    stack.wait_link_up().await;

    defmt::info!("Link up, waiting for DHCP...");
    log::info!("Link up, waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        defmt::info!("Got IP address: {}", Debug2Format(&config.address));
        log::info!("Got IP address: {:?}", config.address);
    }

    defmt::info!("Network ready!");
    log::info!("Network ready!");

    // Initialize I2C bus for sensors
    // Using GPIO4 (SDA) and GPIO5 (SCL)
    let sda = p.PIN_4;
    let scl = p.PIN_5;
    let mut i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, I2cConfig::default());

    // Wait for sensors to power up before scanning
    defmt::info!("Waiting for I2C devices to power up...");
    log::info!("Waiting for I2C devices to power up...");
    Timer::after(Duration::from_millis(2000)).await;

    // Scan I2C bus to find devices
    scan_i2c_bus(&mut i2c).await;

    // Create shared I2C bus
    static I2C_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Create I2C devices for all sensors (shared bus)
    let scd40_i2c = I2cDevice::new(i2c_bus);
    let aht20_i2c = I2cDevice::new(i2c_bus);
    let bmp280_i2c = I2cDevice::new(i2c_bus);

    // Initialize SCD40 CO2 sensor (0x62)
    let mut scd40 = match Scd40Sensor::new(scd40_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize SCD40: {} - halting", e);
            log::error!("Failed to initialize SCD40: {} - halting", e);
            loop {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Initialize AHT20 temperature/humidity sensor (0x38)
    let mut aht20 = match Aht20Sensor::new(aht20_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize AHT20: {} - halting", e);
            log::error!("Failed to initialize AHT20: {} - halting", e);
            loop {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Initialize BMP280 pressure sensor (0x77)
    let mut bmp280 = match Bmp280Sensor::new(bmp280_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize BMP280: {} - halting", e);
            log::error!("Failed to initialize BMP280: {} - halting", e);
            loop {
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Wait for SCD40 warmup period
    defmt::info!("Waiting for SCD40 warmup period of {} seconds", WARMUP_TIME);
    log::info!("Waiting for SCD40 warmup period of {} seconds", WARMUP_TIME);
    defmt::info!("LED will blink slowly during warmup...");
    log::info!("LED will blink slowly during warmup...");

    for i in 0..WARMUP_TIME {
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(200)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(800)).await;

        if (i + 1) % 15 == 0 {
            defmt::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
            log::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
        }
    }

    defmt::info!("Warmup complete - starting continuous readings");
    log::info!("Warmup complete - starting continuous readings");

    // Track network status for reconnection
    let mut consecutive_network_failures: u8 = 0;
    const MAX_FAILURES_BEFORE_RECONNECT: u8 = 3;

    // Main sensor reading loop
    loop {
        // Indicate reading with LED
        control.gpio_set(0, true).await;

        // Read all sensors
        let co2_reading = scd40.wait_and_read_co2().await;
        let temp_humidity_reading = aht20.read().await;
        let pressure_reading = bmp280.read_pressure().await;

        // Combine readings if all successful
        let readings = match (co2_reading, temp_humidity_reading, pressure_reading) {
            (Ok(co2), Ok(th), Ok(pressure_hpa)) => {
                let temperature_c = th.temperature_c;
                let temperature_f = (temperature_c * 1.8) + 32.0;

                defmt::info!(
                    "CO2: {} ppm, Temp: {} F, Humidity: {}%, Pressure: {} hPa",
                    co2, temperature_f, th.humidity_percent, pressure_hpa
                );
                log::info!(
                    "CO2: {} ppm, Temp: {:.1} F, Humidity: {:.1}%, Pressure: {:.1} hPa",
                    co2, temperature_f, th.humidity_percent, pressure_hpa
                );

                Some(SensorReadings {
                    co2,
                    temperature_c,
                    temperature_f,
                    humidity_percent: th.humidity_percent,
                    pressure_hpa,
                })
            }
            (Err(e), _, _) => {
                defmt::warn!("SCD40 read failed: {}", e);
                log::warn!("SCD40 read failed: {}", e);
                None
            }
            (_, Err(e), _) => {
                defmt::warn!("AHT20 read failed: {}", e);
                log::warn!("AHT20 read failed: {}", e);
                None
            }
            (_, _, Err(e)) => {
                defmt::warn!("BMP280 read failed: {}", e);
                log::warn!("BMP280 read failed: {}", e);
                None
            }
        };

        // Check network status and attempt reconnection if needed
        if check_network_status(stack) == NetworkStatus::Disconnected {
            defmt::warn!("Network disconnected, attempting to reconnect...");
            log::warn!("Network disconnected, attempting to reconnect...");

            // Attempt to rejoin WiFi
            match control.join(WIFI_SSID, JoinOptions::new(WIFI_PASSWORD.as_bytes())).await {
                Ok(()) => {
                    defmt::info!("WiFi rejoined, waiting for link...");
                    log::info!("WiFi rejoined, waiting for link...");

                    // Wait for link with timeout
                    let link_timeout = embassy_time::with_timeout(
                        Duration::from_secs(10),
                        stack.wait_link_up(),
                    )
                    .await;

                    if link_timeout.is_ok() {
                        // Wait for DHCP with timeout
                        let dhcp_timeout = embassy_time::with_timeout(
                            Duration::from_secs(15),
                            stack.wait_config_up(),
                        )
                        .await;

                        if dhcp_timeout.is_ok() {
                            if let Some(config) = stack.config_v4() {
                                defmt::info!("Reconnected with IP: {}", Debug2Format(&config.address));
                                log::info!("Reconnected with IP: {:?}", config.address);
                            }
                            consecutive_network_failures = 0;
                        } else {
                            defmt::warn!("DHCP timeout during reconnection");
                            log::warn!("DHCP timeout during reconnection");
                        }
                    } else {
                        defmt::warn!("Link timeout during reconnection");
                        log::warn!("Link timeout during reconnection");
                    }
                }
                Err(e) => {
                    defmt::warn!("WiFi rejoin failed: {}", Debug2Format(&e));
                    log::warn!("WiFi rejoin failed, will retry next cycle");
                }
            }
        }

        // Submit data to API if readings are available
        if let Some(r) = &readings {
            let payload = SensorPayload::new(r);

            match submit_sensor_data(stack, &payload, &mut rng).await {
                Ok(()) => {
                    defmt::debug!("Data submitted to API");
                    consecutive_network_failures = 0;
                }
                Err(e) => {
                    defmt::warn!("API submission failed: {}", e);
                    log::warn!("API submission failed: {}", e);

                    consecutive_network_failures = consecutive_network_failures.saturating_add(1);

                    if consecutive_network_failures >= MAX_FAILURES_BEFORE_RECONNECT {
                        defmt::warn!(
                            "Multiple consecutive failures ({}), will check network next cycle",
                            consecutive_network_failures
                        );
                        log::warn!(
                            "Multiple consecutive failures ({}), will check network next cycle",
                            consecutive_network_failures
                        );
                    }
                }
            }
        }

        control.gpio_set(0, false).await;

        // Wait before next reading
        Timer::after(Duration::from_secs(READ_INTERVAL)).await;
    }
}
