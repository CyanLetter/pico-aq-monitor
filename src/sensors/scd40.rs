//! SCD40 CO2 sensor wrapper.
//!
//! The SCD40 is a true NDIR CO2 sensor that measures actual CO2 concentration,
//! unlike the ENS160 which estimates eCO2 from VOC levels.
//!
//! I2C address: 0x62

use defmt::Debug2Format;
use embassy_time::{Delay, Duration, Timer};
use scd4x::Scd4xAsync;

/// SCD40 I2C address
#[cfg(feature = "scd40-frc")]
const SCD40_I2C_ADDR: u8 = 0x62;

/// Target CO2 concentration for forced recalibration (fresh outdoor air)
#[cfg(feature = "scd40-frc")]
const FRC_TARGET_CO2_PPM: u16 = 428;

/// Duration to run periodic measurements before FRC (3 minutes)
#[cfg(feature = "scd40-frc")]
const FRC_WARMUP_SECS: u64 = 180;

/// Calculate CRC8 for SCD40 (polynomial 0x31, init 0xFF)
#[cfg(feature = "scd40-frc")]
fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Send a command to SCD40 (no data)
#[cfg(feature = "scd40-frc")]
async fn scd40_send_cmd<I2C, E>(i2c: &mut I2C, cmd: u16) -> Result<(), &'static str>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: core::fmt::Debug,
{
    let buf = [(cmd >> 8) as u8, (cmd & 0xFF) as u8];
    i2c.write(SCD40_I2C_ADDR, &buf)
        .await
        .map_err(|_| "I2C write failed")
}

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
    #[cfg(not(feature = "scd40-frc"))]
    pub async fn new(i2c: I2C) -> Result<Self, &'static str> {
        Self::init_sensor(i2c).await
    }

    /// Create and initialize a new SCD40 sensor with forced recalibration
    #[cfg(feature = "scd40-frc")]
    pub async fn new(mut i2c: I2C) -> Result<Self, &'static str> {

        defmt::info!("=== SCD40 FORCED RECALIBRATION MODE ===");
        defmt::info!("Ensure sensor is exposed to fresh outdoor air (~428 ppm CO2)");
        log::info!("=== SCD40 FORCED RECALIBRATION MODE ===");
        log::info!("Ensure sensor is exposed to fresh outdoor air (~428 ppm CO2)");

        // Step 1: Stop any ongoing measurement
        defmt::info!("Stopping any ongoing measurements...");
        let _ = scd40_send_cmd(&mut i2c, 0x3F86).await; // stop_periodic_measurement
        Timer::after(Duration::from_millis(500)).await;

        // Step 2: Reinitialize sensor
        defmt::info!("Reinitializing sensor...");
        scd40_send_cmd(&mut i2c, 0x3646).await?; // reinit
        Timer::after(Duration::from_millis(30)).await;

        // Step 3: Start periodic measurement for warmup
        defmt::info!("Starting periodic measurement for {} second warmup...", FRC_WARMUP_SECS);
        log::info!("Starting periodic measurement for {} second warmup...", FRC_WARMUP_SECS);
        scd40_send_cmd(&mut i2c, 0x21B1).await?; // start_periodic_measurement

        // Step 4: Wait 3 minutes for warmup
        for i in 0..FRC_WARMUP_SECS {
            Timer::after(Duration::from_secs(1)).await;
            if (i + 1) % 30 == 0 {
                defmt::info!("FRC warmup: {} / {} seconds", i + 1, FRC_WARMUP_SECS);
                log::info!("FRC warmup: {} / {} seconds", i + 1, FRC_WARMUP_SECS);
            }
        }

        // Step 5: Stop periodic measurement
        defmt::info!("Stopping periodic measurement for FRC...");
        log::info!("Stopping periodic measurement for FRC...");
        scd40_send_cmd(&mut i2c, 0x3F86).await?; // stop_periodic_measurement

        // Step 6: Wait at least 500ms (using 1s for safety)
        Timer::after(Duration::from_millis(1000)).await;

        // Step 7: Perform forced recalibration
        defmt::info!("Performing forced recalibration with target {} ppm...", FRC_TARGET_CO2_PPM);
        log::info!("Performing forced recalibration with target {} ppm...", FRC_TARGET_CO2_PPM);

        // Command 0x362F: perform_forced_recalibration
        // Format: [cmd_hi, cmd_lo, data_hi, data_lo, crc]
        let data_bytes = FRC_TARGET_CO2_PPM.to_be_bytes();
        let data_crc = crc8(&data_bytes);
        let write_buf: [u8; 5] = [
            0x36, 0x2F,             // Command
            data_bytes[0],          // Target CO2 high byte
            data_bytes[1],          // Target CO2 low byte
            data_crc,               // CRC of target CO2
        ];

        i2c.write(SCD40_I2C_ADDR, &write_buf)
            .await
            .map_err(|e| {
                defmt::error!("FRC I2C write failed: {}", Debug2Format(&e));
                "FRC I2C write failed"
            })?;

        // Step 8: Wait 400ms for FRC to complete (per datasheet)
        Timer::after(Duration::from_millis(400)).await;

        // Step 9: Read 3 bytes response: [correction_hi, correction_lo, crc]
        let mut read_buf = [0u8; 3];
        i2c.read(SCD40_I2C_ADDR, &mut read_buf)
            .await
            .map_err(|e| {
                defmt::error!("FRC I2C read failed: {}", Debug2Format(&e));
                "FRC I2C read failed"
            })?;

        // Verify CRC
        let expected_crc = crc8(&read_buf[0..2]);
        if read_buf[2] != expected_crc {
            defmt::error!("FRC CRC mismatch: got 0x{:02X}, expected 0x{:02X}", read_buf[2], expected_crc);
            return Err("FRC CRC verification failed");
        }

        let frc_correction = u16::from_be_bytes([read_buf[0], read_buf[1]]);
        defmt::info!("FRC raw response: 0x{:04X}", frc_correction);

        // Check if FRC failed (0xFFFF indicates failure)
        if frc_correction == 0xFFFF {
            defmt::error!("SCD40 forced recalibration FAILED (returned 0xFFFF)");
            log::error!("SCD40 forced recalibration FAILED (returned 0xFFFF)");
            return Err("Forced recalibration failed - sensor returned 0xFFFF");
        }

        // FRC correction is in units of 1 ppm, with offset of 0x8000
        // Actual correction = frc_correction - 0x8000
        let correction_ppm = frc_correction as i32 - 0x8000;
        defmt::info!("SCD40 forced recalibration SUCCESS!");
        defmt::info!("FRC correction value: {} ppm (raw: 0x{:04X})", correction_ppm, frc_correction);
        log::info!("SCD40 forced recalibration SUCCESS!");
        log::info!("FRC correction value: {} ppm (raw: 0x{:04X})", correction_ppm, frc_correction);

        defmt::info!("=== FRC COMPLETE - Sensor calibrated ===");
        log::info!("=== FRC COMPLETE - Sensor calibrated ===");

        // Now initialize the sensor normally
        Self::init_sensor(i2c).await
    }

    /// Internal sensor initialization (shared by FRC and non-FRC paths)
    async fn init_sensor(i2c: I2C) -> Result<Self, &'static str> {
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
