//! WS2812B LED array control using PIO.
//!
//! Controls an array of 8 WS2812B LEDs with pulsing animations
//! that change color based on CO2 readings.
//!
//! Uses PIO1 (PIO0 is used by CYW43 WiFi chip).

use embassy_rp::Peri;
use embassy_rp::peripherals::{DMA_CH1, PIN_16, PIO1};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use libm::sinf;
use smart_leds::RGB8;

/// Number of LEDs in the array
pub const NUM_LEDS: usize = 8;

/// CO2 thresholds for color changes (in ppm)
mod thresholds {
    pub const GREEN_MAX: u16 = 450;
    pub const BLUE_MAX: u16 = 600;
    pub const AMBER_MAX: u16 = 900;
}

/// Base colors for each CO2 level
mod colors {
    use smart_leds::RGB8;

    /// Green - excellent air quality (< 450 ppm)
    pub const GREEN: RGB8 = RGB8::new(0, 255, 50);

    /// Cool blue - good air quality (450-600 ppm)
    pub const BLUE: RGB8 = RGB8::new(0, 150, 255);

    /// Amber - moderate air quality (600-900 ppm)
    pub const AMBER: RGB8 = RGB8::new(255, 150, 0);

    /// Red - poor air quality (> 900 ppm)
    pub const RED: RGB8 = RGB8::new(255, 30, 0);
}

/// Get the base color for a given CO2 level
fn co2_to_color(co2_ppm: u16) -> RGB8 {
    if co2_ppm < thresholds::GREEN_MAX {
        colors::GREEN
    } else if co2_ppm < thresholds::BLUE_MAX {
        colors::BLUE
    } else if co2_ppm < thresholds::AMBER_MAX {
        colors::AMBER
    } else {
        colors::RED
    }
}

/// Scale a color by a brightness factor (0.0 to 1.0)
fn scale_color(color: RGB8, brightness: f32) -> RGB8 {
    let brightness = brightness.clamp(0.0, 1.0);
    // defmt::info!("Brightness is {}", brightness);
    let scaled = RGB8::new(
        (color.r as f32 * brightness) as u8,
        (color.g as f32 * brightness) as u8,
        (color.b as f32 * brightness) as u8,
    );
    // defmt::info!("R: {} G: {} B: {}", scaled.r, scaled.g, scaled.b);
    scaled
}

/// Calculate pulsing brightness for a single LED in the sequence.
///
/// Creates an overlapping wave effect where each LED pulses slightly
/// out of phase with its neighbors.
fn pulse_brightness(
    led_index: usize,
    time: f32,
    wave_speed: f32,
    min_brightness: f32,
    max_brightness: f32,
) -> f32 {
    // Phase offset for each LED creates the overlapping wave
    let phase_offset = (led_index as f32 / NUM_LEDS as f32) * core::f32::consts::PI * 2.0;

    // Sinusoidal pulse
    let sin_value = sinf(time * wave_speed - phase_offset);

    // Map from [-1, 1] to [min_brightness, max_brightness]
    let normalized = (sin_value + 1.0) / 2.0;
    min_brightness + normalized * (max_brightness - min_brightness)
}

/// LED controller for WS2812B array using PIO1
pub struct LedController<'d> {
    ws: PioWs2812<'d, PIO1, 0, NUM_LEDS, Grb>,
    current_co2: u16,
    time_counter: f32,
}

impl<'d> LedController<'d> {
    /// Create a new LED controller
    ///
    /// Uses PIO1, state machine 0, DMA_CH1, and PIN_16.
    pub fn new(
        common: &mut Common<'d, PIO1>,
        sm: StateMachine<'d, PIO1, 0>,
        dma: Peri<'d, DMA_CH1>,
        pin: Peri<'d, PIN_16>,
        program: &PioWs2812Program<'d, PIO1>,
    ) -> Self {
        Self {
            ws: PioWs2812::new(common, sm, dma, pin, program),
            current_co2: 400, // Default to good air quality
            time_counter: 0.0,
        }
    }

    /// Update the CO2 reading for color selection
    pub fn set_co2(&mut self, co2_ppm: u16) {
        self.current_co2 = co2_ppm;
    }

    /// Advance animation time and update LEDs
    ///
    /// Call this regularly (e.g., every 50ms) for smooth animation.
    pub async fn update(&mut self, delta_secs: f32) {
        self.time_counter += delta_secs;

        // Wrap time counter to prevent float precision issues
        if self.time_counter > 1000.0 {
            self.time_counter -= 1000.0;
        }

        let base_color = co2_to_color(self.current_co2);
        let mut led_colors = [RGB8::default(); NUM_LEDS];

        // Animation parameters
        const WAVE_SPEED: f32 = 8.0; // Slow, gentle pulse
        const MIN_BRIGHTNESS: f32 = 0.01;
        const MAX_BRIGHTNESS: f32 = 0.2;

        for i in 0..NUM_LEDS {
            let brightness = pulse_brightness(
                i,
                self.time_counter,
                WAVE_SPEED,
                MIN_BRIGHTNESS,
                MAX_BRIGHTNESS,
            );
            led_colors[i] = scale_color(base_color, brightness);
        }

        // defmt::info!("R: {} G: {} B: {}", led_colors[0].r, led_colors[0].g, led_colors[0].b);

        self.ws.write(&led_colors).await;
    }

    /// Turn off all LEDs
    #[allow(dead_code)]
    pub async fn clear(&mut self) {
        let off = [RGB8::default(); NUM_LEDS];
        self.ws.write(&off).await;
    }
}
