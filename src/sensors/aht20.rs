//! AHT20 temperature/humidity sensor wrapper.
//!
//! I2C address: 0x38

use crate::types::TempHumidity;
use aht20_async::Aht20;
use defmt::Debug2Format;
use embassy_time::Delay;

/// AHT20 sensor wrapper
pub struct Aht20Sensor<I2C> {
    sensor: Aht20<I2C, Delay>,
}

impl<I2C, E> Aht20Sensor<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Create and initialize a new AHT20 sensor
    pub async fn new(i2c: I2C) -> Result<Self, &'static str> {
        let sensor = match Aht20::new(i2c, Delay).await {
            Ok(s) => s,
            Err(e) => {
                defmt::error!("AHT20 initialization failed: {}", Debug2Format(&e));
                return Err("Failed to initialize AHT20");
            }
        };

        defmt::info!("AHT20 initialized successfully");

        Ok(Self { sensor })
    }

    /// Read temperature and humidity
    pub async fn read(&mut self) -> Result<TempHumidity, &'static str> {
        let (humidity, temperature) = self
            .sensor
            .read()
            .await
            .map_err(|e| {
                defmt::error!("AHT20 measurement failed: {}", Debug2Format(&e));
                "Failed to read AHT20"
            })?;

        let temperature_c = temperature.celsius();
        let humidity_percent = humidity.rh();

        defmt::debug!("AHT20: {} C, {}% RH", temperature_c, humidity_percent);

        Ok(TempHumidity {
            temperature_c,
            humidity_percent,
        })
    }
}
