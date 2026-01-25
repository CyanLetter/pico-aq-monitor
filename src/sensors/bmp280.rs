//! BMP280 pressure sensor wrapper.
//!
//! Uses the bme280-rs crate which is compatible with BMP280.
//! BMP280 only provides pressure and temperature (no humidity).
//!
//! I2C address: 0x77

use bme280_rs::{AsyncBme280, Configuration, Oversampling, SensorMode};
use defmt::Debug2Format;
use embassy_time::Delay;

/// BMP280 I2C address
const BMP280_ADDRESS: u8 = 0x77;

/// BMP280 sensor wrapper
pub struct Bmp280Sensor<I2C> {
    sensor: AsyncBme280<I2C, Delay>,
}

impl<I2C, E> Bmp280Sensor<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Create and initialize a new BMP280 sensor
    pub async fn new(i2c: I2C) -> Result<Self, &'static str> {
        let mut sensor = AsyncBme280::new_with_address(i2c, BMP280_ADDRESS, Delay);

        if let Err(e) = sensor.init().await {
            defmt::error!("BMP280 initialization failed: {}", Debug2Format(&e));
            return Err("Failed to initialize BMP280");
        }

        // Configure for pressure readings only (temperature is bonus)
        let config = Configuration::default()
            .with_temperature_oversampling(Oversampling::Oversample1)
            .with_pressure_oversampling(Oversampling::Oversample1)
            .with_sensor_mode(SensorMode::Normal);

        if let Err(e) = sensor.set_sampling_configuration(config).await {
            defmt::error!("BMP280 configuration failed: {}", Debug2Format(&e));
            return Err("Failed to configure BMP280");
        }

        defmt::info!("BMP280 initialized successfully");

        Ok(Self { sensor })
    }

    /// Read pressure in hPa
    pub async fn read_pressure(&mut self) -> Result<f32, &'static str> {
        let pressure_pa = self
            .sensor
            .read_pressure()
            .await
            .map_err(|e| {
                defmt::error!("BMP280 pressure read failed: {}", Debug2Format(&e));
                "Failed to read BMP280 pressure"
            })?
            .ok_or("BMP280 pressure not available")?;

        let pressure_hpa = pressure_pa / 100.0; // Convert Pa to hPa
        defmt::debug!("BMP280: {} hPa", pressure_hpa);

        Ok(pressure_hpa)
    }
}
