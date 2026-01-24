//! SCD40 CO2 sensor wrapper.
//!
//! The SCD40 is a true NDIR CO2 sensor that measures actual CO2 concentration,
//! unlike the ENS160 which estimates eCO2 from VOC levels.
//!
//! I2C address: 0x62

use defmt::Debug2Format;
use embassy_time::{Delay, Duration, Timer};
use scd4x::Scd4xAsync;

/// SCD40 sensor wrapper
pub struct Scd40Sensor<I2C> {
    sensor: Scd4xAsync<I2C, Delay>,
}

impl<I2C, E> Scd40Sensor<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    /// Create and initialize a new SCD40 sensor
    pub async fn new(i2c: I2C) -> Result<Self, &'static str> {
        let mut sensor = Scd4xAsync::new(i2c, Delay);

        // Stop any ongoing measurement first
        if let Err(e) = sensor.stop_periodic_measurement().await {
            defmt::warn!("SCD40 stop_periodic_measurement failed (may be normal): {}", Debug2Format(&e));
        }

        // Wait for the sensor to be ready after stop command
        Timer::after(Duration::from_millis(500)).await;

        // Reinitialize the sensor
        if let Err(e) = sensor.reinit().await {
            defmt::error!("SCD40 reinit failed: {}", Debug2Format(&e));
            return Err("Failed to reinitialize SCD40");
        }

        // Wait for reinit to complete
        Timer::after(Duration::from_millis(30)).await;

        // Start periodic measurements
        if let Err(e) = sensor.start_periodic_measurement().await {
            defmt::error!("SCD40 start_periodic_measurement failed: {}", Debug2Format(&e));
            return Err("Failed to start SCD40 periodic measurement");
        }

        defmt::info!("SCD40 initialized successfully");
        log::info!("SCD40 initialized successfully");

        Ok(Self { sensor })
    }

    /// Wait for data to be ready and read CO2 measurement
    ///
    /// Returns CO2 concentration in ppm
    pub async fn wait_and_read_co2(&mut self) -> Result<u16, &'static str> {
        // Poll data_ready_status until data is available
        // SCD40 measures every ~5 seconds in periodic mode
        for _ in 0..60 {
            match self.sensor.data_ready_status().await {
                Ok(true) => {
                    // Data is ready, read the measurement
                    match self.sensor.measurement().await {
                        Ok(measurement) => {
                            defmt::info!("CO2: {} ppm", measurement.co2);
                            log::info!("CO2: {} ppm", measurement.co2);
                            return Ok(measurement.co2);
                        }
                        Err(e) => {
                            defmt::error!("SCD40 measurement read failed: {}", Debug2Format(&e));
                            return Err("Failed to read SCD40 measurement");
                        }
                    }
                }
                Ok(false) => {
                    // Data not ready yet, wait and poll again
                    Timer::after(Duration::from_millis(100)).await;
                }
                Err(e) => {
                    defmt::warn!("SCD40 data_ready_status failed: {}", Debug2Format(&e));
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }

        Err("Timeout waiting for SCD40 data")
    }
}
