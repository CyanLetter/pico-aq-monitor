//! ENS160 Air Quality Sensor with BME280 Compensation for RP Pico W
//!
//! Reads air quality data from an ENS160 sensor and uses BME280 temperature/humidity
//! readings to improve accuracy through environmental compensation.
//! Submits sensor data to a REST API endpoint over WiFi.
//! Logs via USB serial and RTT (for debug probe).

#![no_std]
#![no_main]

use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode};
use core::fmt::Write as _;
use cyw43::JoinOptions;
use cyw43_firmware::{CYW43_43439A0 as FIRMWARE, CYW43_43439A0_CLM as CLM};
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::Debug2Format;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Config as NetConfig, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Async, Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embassy_usb_logger::ReceiverHandler;
use ens160_aq::data::InterruptPinConfig;
use ens160_aq::Ens160;
use heapless::String;
use reqwless::client::HttpClient;
use reqwless::request::{Method, RequestBuilder};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

use embassy_rp::usb::InterruptHandler as UsbInterruptHandler;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

// WiFi configuration - change these to match your network
const WIFI_SSID: &str = "YOUR_WIFI_SSID";
const WIFI_PASSWORD: &str = "YOUR_WIFI_PASSWORD";

// API endpoint for sensor data submission (dummy URL for now)
const API_ENDPOINT: &str = "http://192.168.1.100:8080/api/sensors";

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

/// USB serial logger task
struct Handler;

impl ReceiverHandler for Handler {
    fn new() -> Self {
        Handler
    }

    async fn handle_data(&self, _data: &[u8]) {
        // Ignore incoming data from USB serial
    }
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver, Handler);
}

/// I2C bus type alias
type I2c0Bus = Mutex<NoopRawMutex, I2c<'static, I2C0, Async>>;

/// I2C device type alias (for shared bus access)
type I2cDev = I2cDevice<'static, NoopRawMutex, I2c<'static, I2C0, Async>>;

/// ENS160 device type alias
type Ens160Device = Ens160<I2cDev, Delay>;

/// BME280 device type alias
type Bme280Device = AsyncBme280<I2cDev, Delay>;

/// BME280 I2C address (secondary address 0x77)
const BME280_ADDRESS: u8 = 0x77;

/// Warmup time for ENS160 sensor in seconds
const WARMUP_TIME: u64 = 180;

/// Read interval in seconds
const READ_INTERVAL: u64 = 5;

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

/// Struct to hold BME280 environment readings
struct EnvironmentReadings {
    temperature_c: f32,
    temperature_f: f32,
    humidity_percent: f32,
    pressure_hpa: f32,
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
        defmt::error!("Failed to initialize ENS160: {}", Debug2Format(&e));
        log::error!("Failed to initialize ENS160: {:?}", e);
        return None;
    }
    defmt::info!("ENS160 initialized successfully");
    log::info!("ENS160 initialized successfully");

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
            defmt::info!("ENS160 interrupt pin configured: {}", val);
            log::info!("ENS160 interrupt pin configured: {}", val);
        }
        Err(e) => {
            defmt::error!("Failed to configure ENS160 interrupt pin: {}", Debug2Format(&e));
            log::error!("Failed to configure ENS160 interrupt pin: {:?}", e);
            return None;
        }
    }

    Some(ens160)
}

/// Initialize the BME280 sensor
async fn initialize_bme280(i2c_device: I2cDev) -> Option<Bme280Device> {
    let mut bme280 = AsyncBme280::new_with_address(i2c_device, BME280_ADDRESS, Delay);

    if let Err(e) = bme280.init().await {
        defmt::error!("Failed to initialize BME280: {}", Debug2Format(&e));
        log::error!("Failed to initialize BME280: {:?}", e);
        return None;
    }
    defmt::info!("BME280 initialized successfully");
    log::info!("BME280 initialized successfully");

    // Configure sampling for temperature, humidity, and pressure
    let config = Configuration::default()
        .with_temperature_oversampling(Oversampling::Oversample1)
        .with_humidity_oversampling(Oversampling::Oversample1)
        .with_pressure_oversampling(Oversampling::Oversample1)
        .with_sensor_mode(SensorMode::Normal);

    if let Err(e) = bme280.set_sampling_configuration(config).await {
        defmt::error!("Failed to configure BME280: {}", Debug2Format(&e));
        log::error!("Failed to configure BME280: {:?}", e);
        return None;
    }
    defmt::info!("BME280 configured successfully");
    log::info!("BME280 configured successfully");

    Some(bme280)
}

/// Read environment data from BME280 sensor
async fn read_bme280(bme280: &mut Bme280Device) -> Result<EnvironmentReadings, &'static str> {
    let temperature = bme280
        .read_temperature()
        .await
        .map_err(|_| "Failed to read BME280 temperature")?
        .ok_or("BME280 temperature not available")?;

    let humidity = bme280
        .read_humidity()
        .await
        .map_err(|_| "Failed to read BME280 humidity")?
        .ok_or("BME280 humidity not available")?;

    let pressure = bme280
        .read_pressure()
        .await
        .map_err(|_| "Failed to read BME280 pressure")?
        .ok_or("BME280 pressure not available")?;

    Ok(EnvironmentReadings {
        temperature_c: temperature,
        temperature_f: (temperature * 1.8) + 32.0,
        humidity_percent: humidity,
        pressure_hpa: pressure / 100.0, // Convert Pa to hPa
    })
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

    defmt::info!(
        "AQI: {}, eCO2: {} ppm, Ethanol: {} ppb, VOCs: {} ppb",
        readings.aqi, readings.eco2, readings.etoh, readings.tvoc
    );
    log::info!(
        "AQI: {}, eCO2: {} ppm, Ethanol: {} ppb, VOCs: {} ppb",
        readings.aqi, readings.eco2, readings.etoh, readings.tvoc
    );

    Ok(readings)
}

/// Combined sensor data for API submission
struct SensorPayload {
    temperature_c: f32,
    temperature_f: f32,
    humidity_percent: f32,
    pressure_hpa: f32,
    eco2: u16,
    tvoc: u16,
    aqi: u8,
}

impl SensorPayload {
    /// Create a new sensor payload from environment and air quality readings
    fn new(env: &EnvironmentReadings, aq: &Ens160Readings) -> Self {
        Self {
            temperature_c: env.temperature_c,
            temperature_f: env.temperature_f,
            humidity_percent: env.humidity_percent,
            pressure_hpa: env.pressure_hpa,
            eco2: aq.eco2,
            tvoc: aq.tvoc,
            aqi: aq.aqi,
        }
    }

    /// Serialize payload to JSON string
    fn to_json(&self) -> String<512> {
        let mut json: String<512> = String::new();
        // Manual JSON construction (no_std compatible)
        let _ = write!(
            json,
            r#"{{"temperature_c":{:.2},"temperature_f":{:.2},"humidity_percent":{:.2},"pressure_hpa":{:.2},"eco2":{},"tvoc":{},"aqi":{}}}"#,
            self.temperature_c,
            self.temperature_f,
            self.humidity_percent,
            self.pressure_hpa,
            self.eco2,
            self.tvoc,
            self.aqi
        );
        json
    }
}

/// Submit sensor data to API endpoint
async fn submit_sensor_data(
    stack: embassy_net::Stack<'static>,
    payload: &SensorPayload,
) -> Result<(), &'static str> {
    let mut rx_buffer = [0; 1024];

    let client_state = TcpClientState::<1, 1024, 1024>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns_client = DnsSocket::new(stack);

    let mut http_client = HttpClient::new(&tcp_client, &dns_client);

    let json = payload.to_json();
    let json_bytes = json.as_bytes();

    defmt::debug!("Submitting to API: {}", json.as_str());

    let request = http_client
        .request(Method::POST, API_ENDPOINT)
        .await
        .map_err(|_| "Failed to create HTTP request")?;

    let headers = [("Content-Type", "application/json")];
    let mut request = request.headers(&headers).body(json_bytes);

    let response = request
        .send(&mut rx_buffer)
        .await
        .map_err(|_| "Failed to send HTTP request")?;

    let status = response.status.0;
    if status >= 200 && status < 300 {
        defmt::info!("API submission successful (status {})", status);
        log::info!("API submission successful (status {})", status);
        Ok(())
    } else {
        defmt::warn!("API submission failed (status {})", status);
        log::warn!("API submission failed (status {})", status);
        Err("API returned error status")
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize random number generator for network stack
    let mut rng = RoscRng;

    // Initialize USB serial logger
    let usb_driver = Driver::new(p.USB, Irqs);
    spawner.spawn(logger_task(usb_driver)).unwrap();

    // Give USB time to enumerate before logging
    Timer::after(Duration::from_millis(100)).await;

    defmt::info!("ENS160 Air Quality Sensor with WiFi starting...");
    log::info!("ENS160 Air Quality Sensor with WiFi starting...");

    // Initialize CYW43 (WiFi chip) for onboard LED and networking
    // Firmware provided by cyw43-firmware crate
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
    // Using GPIO4 (SDA) and GPIO5 (SCL) - adjust pins as needed for your wiring
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

    // Create I2C devices for both sensors (shared bus)
    let ens160_i2c = I2cDevice::new(i2c_bus);
    let bme280_i2c = I2cDevice::new(i2c_bus);

    // ENS160 interrupt pin - adjust as needed for your wiring
    let mut ens160_int = Input::new(p.PIN_6, Pull::Up);

    // Initialize BME280 first (for environmental compensation)
    let mut bme280 = match initialize_bme280(bme280_i2c).await {
        Some(sensor) => sensor,
        None => {
            defmt::error!("Failed to initialize BME280 - halting");
            log::error!("Failed to initialize BME280 - halting");
            loop {
                // Blink LED rapidly to indicate error
                control.gpio_set(0, true).await;
                Timer::after(Duration::from_millis(100)).await;
                control.gpio_set(0, false).await;
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    // Initialize ENS160
    let mut ens160 = match initialize_ens160(ens160_i2c).await {
        Some(sensor) => sensor,
        None => {
            defmt::error!("Failed to initialize ENS160 - halting");
            log::error!("Failed to initialize ENS160 - halting");
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
    defmt::info!("Waiting for ENS160 warmup period of {} seconds", WARMUP_TIME);
    log::info!("Waiting for ENS160 warmup period of {} seconds", WARMUP_TIME);
    defmt::info!("LED will blink slowly during warmup...");
    log::info!("LED will blink slowly during warmup...");

    for i in 0..WARMUP_TIME {
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(200)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(800)).await;

        if (i + 1) % 30 == 0 {
            defmt::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
            log::info!("Warmup: {} / {} seconds", i + 1, WARMUP_TIME);
        }
    }

    defmt::info!("Warmup complete - starting continuous readings");
    log::info!("Warmup complete - starting continuous readings");

    // Main sensor reading loop
    loop {
        // Indicate reading with LED
        control.gpio_set(0, true).await;

        // Store readings for API submission
        let mut env_readings: Option<EnvironmentReadings> = None;
        let mut aq_readings: Option<Ens160Readings> = None;

        // Read BME280 for environmental data
        match read_bme280(&mut bme280).await {
            Ok(env) => {
                defmt::info!(
                    "Environment: {} F, {}% RH, {} hPa",
                    env.temperature_f, env.humidity_percent, env.pressure_hpa
                );
                log::info!(
                    "Environment: {:.1} F, {:.1}% RH, {:.1} hPa",
                    env.temperature_f, env.humidity_percent, env.pressure_hpa
                );

                // Apply temperature and humidity compensation to ENS160
                let humidity_int = env.humidity_percent as u16;
                if let Err(e) = ens160.set_temp_rh_comp(env.temperature_c, humidity_int).await {
                    defmt::warn!("Failed to set ENS160 compensation: {}", Debug2Format(&e));
                    log::warn!("Failed to set ENS160 compensation: {:?}", e);
                }

                env_readings = Some(env);
            }
            Err(e) => {
                defmt::warn!("BME280 read failed: {}", e);
                log::warn!("BME280 read failed: {}", e);
            }
        }

        // Read ENS160 air quality data
        match read_ens160(&mut ens160, &mut ens160_int).await {
            Ok(readings) => {
                defmt::debug!("Sensor read successful");
                aq_readings = Some(readings);
            }
            Err(e) => {
                defmt::error!("ENS160 read failed: {}", e);
                log::error!("ENS160 read failed: {}", e);
            }
        }

        // Submit data to API if both readings are available
        if let (Some(env), Some(aq)) = (&env_readings, &aq_readings) {
            let payload = SensorPayload::new(env, aq);

            match submit_sensor_data(stack, &payload).await {
                Ok(()) => {
                    defmt::debug!("Data submitted to API");
                }
                Err(e) => {
                    defmt::warn!("API submission failed: {}", e);
                    log::warn!("API submission failed: {}", e);
                }
            }
        }

        control.gpio_set(0, false).await;

        // Wait before next reading
        Timer::after(Duration::from_secs(READ_INTERVAL)).await;
    }
}
