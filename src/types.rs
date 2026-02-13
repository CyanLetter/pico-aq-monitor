//! Shared types for sensor readings and network status.

use core::fmt::Write as _;
use defmt::Format;
use heapless::String;

/// Combined sensor readings from all sensors
#[derive(Clone, Copy, Format)]
pub struct SensorReadings {
    pub co2: u16,
    pub temperature_c: f32,
    pub temperature_f: f32,
    pub humidity_percent: f32,
    pub pressure_hpa: f32,
    pub brightness: u16,
}

/// Sensor payload for API submission
pub struct SensorPayload {
    pub temperature_c: f32,
    pub temperature_f: f32,
    pub humidity_percent: f32,
    pub pressure_hpa: f32,
    pub co2: u16,
    pub brightness: u16,
}

impl SensorPayload {
    /// Create a new sensor payload from sensor readings
    pub fn new(readings: &SensorReadings) -> Self {
        Self {
            temperature_c: readings.temperature_c,
            temperature_f: readings.temperature_f,
            humidity_percent: readings.humidity_percent,
            pressure_hpa: readings.pressure_hpa,
            co2: readings.co2,
            brightness: readings.brightness,
        }
    }

    /// Serialize payload to JSON string
    pub fn to_json(&self) -> String<512> {
        let mut json: String<512> = String::new();
        let _ = write!(
            json,
            r#"{{"temperature_c":{:.2},"temperature_f":{:.2},"humidity_percent":{:.2},"pressure_hpa":{:.2},"co2":{},"brightness":{}}}"#,
            self.temperature_c,
            self.temperature_f,
            self.humidity_percent,
            self.pressure_hpa,
            self.co2,
            self.brightness
        );
        json
    }
}

/// Network status for tracking connectivity
#[derive(Clone, Copy, PartialEq)]
pub enum NetworkStatus {
    Connected,
    Disconnected,
}

/// Temperature and humidity readings from AHT20
#[derive(Clone, Copy)]
pub struct TempHumidity {
    pub temperature_c: f32,
    pub humidity_percent: f32,
}
