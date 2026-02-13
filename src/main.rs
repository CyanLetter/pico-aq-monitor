//! SCD40 CO2 Sensor with AHT20/BMP280 for RP Pico W
//!
//! Reads CO2 data from an SCD40 sensor, temperature/humidity from AHT20,
//! and pressure from BMP280. Submits sensor data to a REST API endpoint over WiFi.
//! WS2812B LED array displays CO2 levels with pulsing animations.
//! Logs via RTT (requires debug probe).
//!
//! Architecture:
//! - Main task: LED animation + sensor reading (never blocks on network)
//! - Network task: Handles API submissions independently
//! - Watchdog: Resets device if main loop hangs

#![no_std]
#![no_main]

extern crate alloc;

mod leds;
mod network;
mod sensors;
mod types;

use embedded_alloc::LlffHeap as Heap;
use portable_atomic::{AtomicU32, Ordering};

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
use embassy_rp::adc::{Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Async as I2cAsync, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::PioWs2812Program;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use static_cell::StaticCell;

use crate::leds::{ButtonState, LedController};
use crate::network::{check_network_status, submit_sensor_data};
use crate::sensors::{Aht20Sensor, Bmp280Sensor, Scd40Sensor};
use crate::types::{NetworkStatus, SensorPayload, SensorReadings};

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

// WiFi configuration loaded from .env file at build time
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

/// Warmup time for SCD40 sensor in seconds (faster than ENS160's 180s)
const WARMUP_TIME: u64 = 60;

/// LED update interval in milliseconds (for smooth animation)
const LED_UPDATE_MS: u64 = 50;

/// Sensor read interval in seconds (for LED color updates)
const SENSOR_READ_SECS: u64 = 5;

/// API submission interval in seconds
const API_SUBMIT_SECS: u64 = 120;

/// Health logging interval in seconds
const HEALTH_LOG_SECS: u64 = 300; // Every 5 minutes

/// Watchdog timeout in milliseconds (must feed within this time)
const WATCHDOG_TIMEOUT_MS: u64 = 8000;

/// Channel for sending sensor readings to network task
static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, SensorReadings, 2> = Channel::new();

/// Uptime counter in seconds (atomic for cross-task access)
static UPTIME_SECS: AtomicU32 = AtomicU32::new(0);

/// I2C bus type alias
type I2c0Bus = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, I2cAsync>>;

/// Attempt to recover a stuck I2C bus by toggling SCL
///
/// I2C slaves can get stuck holding SDA low if a transaction is interrupted.
/// Toggling SCL up to 9 times allows the slave to complete its byte and release SDA.
fn recover_i2c_bus(scl_pin: &mut Output<'_>, _sda_pin: &Output<'_>) {
    // Check if SDA is stuck low
    // Note: We can't easily read SDA state here, so we just do the recovery sequence
    defmt::info!("Attempting I2C bus recovery...");

    // Temporarily use SCL as output to send clock pulses
    // Toggle SCL up to 9 times (one byte + ACK)
    for i in 0..9 {
        // SCL low
        scl_pin.set_low();
        cortex_m::asm::delay(1000); // ~10us at 125MHz

        // SCL high
        scl_pin.set_high();
        cortex_m::asm::delay(1000);

        defmt::trace!("I2C recovery pulse {}", i + 1);
    }

    // Generate STOP condition: SDA low, then SCL high, then SDA high
    // This is approximate since we don't control SDA here
    defmt::info!("I2C bus recovery complete");
}

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

/// Network submission task - runs independently of main loop
///
/// Receives sensor readings via channel and submits to API.
/// This allows LED animations to continue during network operations.
#[embassy_executor::task]
async fn api_submit_task(stack: embassy_net::Stack<'static>) {
    defmt::info!("API submission task started");

    let mut rng = RoscRng;
    let mut consecutive_failures: u8 = 0;

    loop {
        // Wait for sensor readings from the channel
        let readings = SENSOR_CHANNEL.receive().await;

        defmt::debug!("Network task received readings: CO2={} ppm", readings.co2);

        // Check network status
        if check_network_status(stack) == NetworkStatus::Disconnected {
            defmt::warn!("Network disconnected, skipping API submission");
            consecutive_failures = consecutive_failures.saturating_add(1);
            continue;
        }

        // Submit to API
        let payload = SensorPayload::new(&readings);
        match submit_sensor_data(stack, &payload, &mut rng).await {
            Ok(()) => {
                defmt::debug!("API submission successful");
                consecutive_failures = 0;
            }
            Err(e) => {
                consecutive_failures = consecutive_failures.saturating_add(1);
                defmt::warn!(
                    "API submission failed (attempt {}): {}",
                    consecutive_failures,
                    e
                );
            }
        }
    }
}

/// Uptime tracking task - increments uptime counter every second
#[embassy_executor::task]
async fn uptime_task() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        UPTIME_SECS.fetch_add(1, Ordering::Relaxed);
    }
}

/// Log system health information
fn log_health() {
    let uptime = UPTIME_SECS.load(Ordering::Relaxed);
    let hours = uptime / 3600;
    let minutes = (uptime % 3600) / 60;
    let seconds = uptime % 60;

    defmt::info!(
        "Health: uptime={}h {}m {}s",
        hours,
        minutes,
        seconds
    );

    // Log heap usage if we can get it
    // Note: embedded-alloc doesn't provide easy introspection,
    // but we log that the system is still running
    defmt::info!("Health: main loop responsive, watchdog fed");
}

/// Scan I2C bus for devices and log results
async fn scan_i2c_bus(i2c: &mut I2c<'static, I2C0, I2cAsync>) {
    use embedded_hal_async::i2c::I2c as _;

    defmt::info!("Scanning I2C bus...");

    let mut found_count = 0;

    // Scan valid 7-bit addresses (0x08-0x77, excluding reserved ranges)
    for addr in 0x08u8..=0x77u8 {
        // Try to read a single byte from each address
        let mut buf = [0u8; 1];
        if i2c.read(addr, &mut buf).await.is_ok() {
            defmt::info!("  Found device at address 0x{:02X}", addr);
            found_count += 1;
        }
    }

    if found_count == 0 {
        defmt::warn!("No I2C devices found!");
    } else {
        defmt::info!("Found {} device(s)", found_count);
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

    // Initialize hardware watchdog
    // If not fed within WATCHDOG_TIMEOUT_MS, the device will reset
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(WATCHDOG_TIMEOUT_MS));
    defmt::info!("Watchdog started with {}ms timeout", WATCHDOG_TIMEOUT_MS);

    // Feed watchdog immediately
    watchdog.feed();

    // Initialize random number generator for network stack
    let mut rng = RoscRng;

    defmt::info!("SCD40 CO2 Sensor with WiFi starting...");
    defmt::info!("Build features: watchdog, separate network task, I2C recovery");

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

    watchdog.feed();

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    watchdog.feed();

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

    // Start uptime tracking task
    spawner.spawn(uptime_task()).unwrap();

    // Connect to WiFi
    defmt::info!("Connecting to WiFi network: {}", WIFI_SSID);

    loop {
        watchdog.feed();
        match control
            .join(WIFI_SSID, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
            .await
        {
            Ok(()) => break,
            Err(e) => {
                defmt::warn!("WiFi join failed: {}", Debug2Format(&e));
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    defmt::info!("WiFi connected, waiting for link...");
    watchdog.feed();

    // Wait for link with timeout and watchdog feeding
    let link_start = Instant::now();
    while !stack.is_link_up() {
        if link_start.elapsed() > Duration::from_secs(30) {
            defmt::error!("Link timeout - resetting via watchdog");
            loop {} // Watchdog will reset
        }
        watchdog.feed();
        Timer::after(Duration::from_millis(100)).await;
    }

    defmt::info!("Link up, waiting for DHCP...");

    // Wait for DHCP with timeout and watchdog feeding
    let dhcp_start = Instant::now();
    while stack.config_v4().is_none() {
        if dhcp_start.elapsed() > Duration::from_secs(30) {
            defmt::error!("DHCP timeout - resetting via watchdog");
            loop {} // Watchdog will reset
        }
        watchdog.feed();
        Timer::after(Duration::from_millis(100)).await;
    }

    if let Some(config) = stack.config_v4() {
        defmt::info!("Got IP address: {}", Debug2Format(&config.address));
    }

    defmt::info!("Network ready!");
    watchdog.feed();

    // Start the API submission task (runs independently)
    spawner.spawn(api_submit_task(stack)).unwrap();

    // Initialize PIO1 for WS2812B LEDs (PIO0 is used by CYW43 WiFi)
    // Using GPIO16 for LED data line (single wire)
    let Pio {
        common: mut pio1_common,
        sm0: pio1_sm0,
        ..
    } = Pio::new(p.PIO1, Irqs);
    let ws2812_program = PioWs2812Program::new(&mut pio1_common);
    let mut led_controller = LedController::new(
        &mut pio1_common,
        pio1_sm0,
        p.DMA_CH1,
        p.PIN_16,
        &ws2812_program,
    );

    defmt::info!("WS2812B LED controller initialized on GPIO16");
    watchdog.feed();

    // Initialize ADC for LDR photoresistor on GPIO28 (ADC2)
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut ldr_channel = AdcChannel::new_pin(p.PIN_28, Pull::None);
    let mut smoothed_adc: f32 = 2048.0; // Start at midpoint
    defmt::info!("LDR photoresistor initialized on GPIO28 (ADC2)");

    // Initialize button on GPIO1 (active low with internal pull-up)
    let button = Input::new(p.PIN_1, Pull::Up);
    let mut button_press_start: Option<Instant> = None;
    defmt::info!("Button initialized on GPIO1");

    // Prepare I2C pins for potential bus recovery
    // GPIO4 (SDA) and GPIO5 (SCL)
    let mut scl_pin = Output::new(p.PIN_5, Level::High);
    let sda_pin = Output::new(p.PIN_4, Level::High);

    // Perform I2C bus recovery before initializing
    recover_i2c_bus(&mut scl_pin, &sda_pin);

    // Convert pins back to I2C function
    // We need to drop the Output wrappers and reinitialize
    let sda = unsafe { embassy_rp::peripherals::PIN_4::steal() };
    let scl = unsafe { embassy_rp::peripherals::PIN_5::steal() };

    let mut i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, I2cConfig::default());

    // Wait for sensors to power up before scanning
    defmt::info!("Waiting for I2C devices to power up...");
    Timer::after(Duration::from_millis(2000)).await;
    watchdog.feed();

    // Scan I2C bus to find devices
    scan_i2c_bus(&mut i2c).await;
    watchdog.feed();

    // Create shared I2C bus
    static I2C_BUS: StaticCell<I2c0Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Create I2C devices for all sensors (shared bus)
    let scd40_i2c = I2cDevice::new(i2c_bus);
    let aht20_i2c = I2cDevice::new(i2c_bus);
    let bmp280_i2c = I2cDevice::new(i2c_bus);

    // Initialize SCD40 CO2 sensor (0x62)
    watchdog.feed();
    let mut scd40 = match Scd40Sensor::new(scd40_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize SCD40: {} - halting", e);
            loop {
                watchdog.feed(); // Keep feeding so we can debug
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Initialize AHT20 temperature/humidity sensor (0x38)
    watchdog.feed();
    let mut aht20 = match Aht20Sensor::new(aht20_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize AHT20: {} - halting", e);
            loop {
                watchdog.feed();
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Initialize BMP280 pressure sensor (0x77)
    watchdog.feed();
    let mut bmp280 = match Bmp280Sensor::new(bmp280_i2c).await {
        Ok(sensor) => sensor,
        Err(e) => {
            defmt::error!("Failed to initialize BMP280: {} - halting", e);
            loop {
                watchdog.feed();
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Wait for SCD40 warmup period with LED animation
    defmt::info!(
        "Waiting for SCD40 warmup period of {} seconds",
        WARMUP_TIME
    );
    defmt::info!("LEDs will pulse during warmup...");

    // LED animation during warmup (default CO2 level shows green)
    let warmup_iterations = (WARMUP_TIME * 1000) / LED_UPDATE_MS;
    for i in 0..warmup_iterations {
        watchdog.feed();

        // Read ambient light and update brightness
        if let Ok(raw) = adc.read(&mut ldr_channel).await {
            smoothed_adc = smoothed_adc * 0.95 + raw as f32 * 0.05;
            led_controller.set_ambient_light(smoothed_adc as u16);
        }

        // Handle button state
        if button.is_low() {
            let start = *button_press_start.get_or_insert(Instant::now());
            if start.elapsed() > Duration::from_secs(3) {
                led_controller.set_button_state(ButtonState::LongPressed);
            } else {
                led_controller.set_button_state(ButtonState::Pressed);
            }
        } else if button_press_start.take().is_some() {
            led_controller.set_button_state(ButtonState::Released);
        }

        let _ = led_controller.update(LED_UPDATE_MS as f32 / 1000.0).await;
        Timer::after(Duration::from_millis(LED_UPDATE_MS)).await;

        // Log progress every 15 seconds
        let elapsed_secs = (i * LED_UPDATE_MS) / 1000;
        if elapsed_secs > 0 && elapsed_secs % 15 == 0 && (i * LED_UPDATE_MS) % 1000 == 0 {
            defmt::info!("Warmup: {} / {} seconds", elapsed_secs, WARMUP_TIME);
        }
    }

    defmt::info!("Warmup complete - starting continuous readings");
    log_health();

    // Timing counters (in milliseconds)
    let mut ms_since_sensor_read: u64 = SENSOR_READ_SECS * 1000; // Trigger immediate read
    let mut ms_since_api_submit: u64 = API_SUBMIT_SECS * 1000; // Trigger immediate submit
    let mut ms_since_health_log: u64 = 0;

    // Track consecutive sensor failures for I2C recovery
    let mut consecutive_i2c_failures: u8 = 0;
    const MAX_I2C_FAILURES: u8 = 5;

    // Latest sensor readings for API submission
    let mut latest_readings: Option<SensorReadings> = None;

    // Main loop - updates LEDs frequently, reads sensors periodically
    // Network submission happens in separate task via channel
    loop {
        // Feed watchdog at the start of each iteration
        watchdog.feed();

        // Read ambient light and update brightness
        if let Ok(raw) = adc.read(&mut ldr_channel).await {
            smoothed_adc = smoothed_adc * 0.95 + raw as f32 * 0.05;
            led_controller.set_ambient_light(smoothed_adc as u16);
        }

        // Handle button state
        if button.is_low() {
            let start = *button_press_start.get_or_insert(Instant::now());
            if start.elapsed() > Duration::from_secs(3) {
                led_controller.set_button_state(ButtonState::LongPressed);
            } else {
                led_controller.set_button_state(ButtonState::Pressed);
            }
        } else if button_press_start.take().is_some() {
            led_controller.set_button_state(ButtonState::Released);
        }

        // Update LED animation (this should never block significantly)
        let _ = led_controller.update(LED_UPDATE_MS as f32 / 1000.0).await;

        // Check if it's time to read sensors
        if ms_since_sensor_read >= SENSOR_READ_SECS * 1000 {
            ms_since_sensor_read = 0;

            // Indicate reading with onboard LED
            control.gpio_set(0, true).await;

            // Read all sensors with timeout protection
            let sensor_timeout = Duration::from_secs(5);

            let co2_result = embassy_time::with_timeout(
                sensor_timeout,
                scd40.wait_and_read_co2(),
            )
            .await;

            let co2_reading = match co2_result {
                Ok(result) => result,
                Err(_) => {
                    defmt::error!("SCD40 read timed out!");
                    Err("Sensor timeout")
                }
            };

            let temp_humidity_reading = aht20.read().await;
            let pressure_reading = bmp280.read_pressure().await;

            // Combine readings if all successful
            let read_success = matches!(
                (&co2_reading, &temp_humidity_reading, &pressure_reading),
                (Ok(_), Ok(_), Ok(_))
            );

            if read_success {
                consecutive_i2c_failures = 0;
            } else {
                consecutive_i2c_failures = consecutive_i2c_failures.saturating_add(1);
                defmt::warn!(
                    "Sensor read failure ({}/{})",
                    consecutive_i2c_failures,
                    MAX_I2C_FAILURES
                );

                if consecutive_i2c_failures >= MAX_I2C_FAILURES {
                    defmt::error!(
                        "Too many consecutive I2C failures - watchdog will reset device"
                    );
                    // Stop feeding watchdog to trigger reset
                    loop {
                        Timer::after(Duration::from_secs(10)).await;
                    }
                }
            }

            latest_readings = match (co2_reading, temp_humidity_reading, pressure_reading) {
                (Ok(co2), Ok(th), Ok(pressure_hpa)) => {
                    let temperature_c = th.temperature_c;
                    let temperature_f = (temperature_c * 1.8) + 32.0;

                    defmt::info!(
                        "CO2: {} ppm, Temp: {} F, Humidity: {}%, Pressure: {} hPa",
                        co2,
                        temperature_f,
                        th.humidity_percent,
                        pressure_hpa
                    );

                    // Update LED color based on CO2 reading
                    led_controller.set_co2(co2);

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
                    latest_readings // Keep previous readings
                }
                (_, Err(e), _) => {
                    defmt::warn!("AHT20 read failed: {}", e);
                    latest_readings
                }
                (_, _, Err(e)) => {
                    defmt::warn!("BMP280 read failed: {}", e);
                    latest_readings
                }
            };

            control.gpio_set(0, false).await;
        }

        // Check if it's time to submit to API (via channel to network task)
        if ms_since_api_submit >= API_SUBMIT_SECS * 1000 {
            ms_since_api_submit = 0;

            // Send readings to network task via channel (non-blocking try_send)
            if let Some(readings) = latest_readings {
                match SENSOR_CHANNEL.try_send(readings) {
                    Ok(()) => {
                        defmt::debug!("Sent readings to network task");
                    }
                    Err(_) => {
                        defmt::warn!("Network task channel full, skipping submission");
                    }
                }
            }
        }

        // Periodic health logging
        if ms_since_health_log >= HEALTH_LOG_SECS * 1000 {
            ms_since_health_log = 0;
            log_health();
        }

        // Wait for next LED update cycle
        Timer::after(Duration::from_millis(LED_UPDATE_MS)).await;
        ms_since_sensor_read += LED_UPDATE_MS;
        ms_since_api_submit += LED_UPDATE_MS;
        ms_since_health_log += LED_UPDATE_MS;
    }
}
