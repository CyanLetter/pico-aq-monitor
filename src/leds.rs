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
use libm::{powf, sinf};
use smart_leds::RGB8;

/// Number of LEDs in the array
pub const NUM_LEDS: usize = 16;

/// Gamma correction value for WS2812B LEDs
///
/// LEDs have a non-linear brightness response. Gamma correction compensates
/// for this, making perceived brightness changes more uniform across the range.
/// A value of 2.0 is typical for WS2812B LEDs.
const GAMMA: f32 = 2.0;

/// CO2 thresholds for color changes (in ppm)
mod thresholds {
    pub const BEST_MAX: u16 = 500;
    pub const OK_MAX: u16 = 900;
}

/// Base colors for each CO2 level
mod colors {
    use smart_leds::RGB8;

    /// Green - excellent air quality (< 500 ppm)
    pub const COLOR_BEST: RGB8 = RGB8::new(0, 255, 50);

    /// Amber - Acceptable air quality (500-900 ppm)
    pub const COLOR_OK: RGB8 = RGB8::new(255, 160, 50);

    /// Red - poor air quality (> 900 ppm)
    pub const COLOR_BAD: RGB8 = RGB8::new(255, 30, 0);

    /// Purple - button pressed indicator
    pub const PURPLE: RGB8 = RGB8::new(128, 0, 255);

    /// Bright white - button long-press indicator
    pub const WHITE: RGB8 = RGB8::new(255, 255, 255);
}

/// Button press state for LED color override
#[derive(Clone, Copy, PartialEq)]
pub enum ButtonState {
    /// Button not pressed - normal CO2-based colors
    Released,
    /// Button pressed - show purple
    Pressed,
    /// Button held for 3+ seconds - show bright white
    LongPressed,
}

/// Map an ADC reading from an LDR photoresistor to maximum LED brightness.
///
/// Assumes a voltage divider where higher ADC values = more ambient light.
/// Brighter environments allow higher LED brightness; darker environments
/// reduce brightness to avoid being blinding.
///
/// ADC input range: 0-4095 (12-bit). Output: 0.3-1.0.
///
/// Only fades below the midpoint (~2048). Above that, LEDs run at full brightness.
///
/// - ADC 0-500 (dark): fixed at 0.3 (floor).
/// - ADC 500-2048 (dim to moderate): linearly interpolates from 0.3 to 1.0.
///   `t` normalizes the ADC value within this band to 0.0-1.0, then
///   `0.3 + t * 0.7` scales from the floor up to full brightness.
/// - ADC 2048-4095 (bright): fixed at 1.0 (full brightness).
pub fn adc_to_max_brightness(adc_value: u16) -> f32 {
    let adc = adc_value.min(4095) as f32;

    if adc < 500.0 {
        // Dark environment — hold at minimum brightness floor
        0.3
    } else if adc < 2048.0 {
        // Dim-to-moderate — linear ramp from 0.3 to 1.0
        // t = 0.0 at ADC 500, t = 1.0 at ADC 2048
        let t = (adc - 500.0) / (2048.0 - 500.0);
        0.3 + t * (1.0 - 0.3)
    } else {
        // Bright environment — full brightness
        1.0
    }
}

/// Get the base color for a given CO2 level
fn co2_to_color(co2_ppm: u16) -> RGB8 {
    if co2_ppm < thresholds::BEST_MAX {
        colors::COLOR_BEST
    } else if co2_ppm < thresholds::OK_MAX {
        colors::COLOR_OK
    } else {
        colors::COLOR_BAD
    }
}

/// Apply gamma correction to a single channel value
///
/// Takes a linear value (0-255) and returns a gamma-corrected value.
/// This compensates for the non-linear brightness response of LEDs,
/// making perceived brightness changes more uniform.
#[inline]
fn gamma_correct_channel(value: u8) -> u8 {
    // Normalize to 0.0-1.0, apply gamma, scale back to 0-255
    let normalized = value as f32 / 255.0;
    let corrected = powf(normalized, GAMMA);
    (corrected * 255.0) as u8
}

/// Scale a color by a brightness factor (0.0 to 1.0) with gamma correction
///
/// Applies brightness scaling first, then gamma correction to each channel.
/// This produces perceptually uniform brightness transitions.
fn scale_color(color: RGB8, brightness: f32) -> RGB8 {
    let brightness = brightness.clamp(0.0, 1.0);

    // Scale by brightness first (linear)
    let r_linear = (color.r as f32 * brightness) as u8;
    let g_linear = (color.g as f32 * brightness) as u8;
    let b_linear = (color.b as f32 * brightness) as u8;

    // Apply gamma correction to each channel
    RGB8::new(
        gamma_correct_channel(r_linear),
        gamma_correct_channel(g_linear),
        gamma_correct_channel(b_linear),
    )
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
    max_brightness: f32,
    override_color: Option<RGB8>,
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
            max_brightness: 1.0,
            override_color: None,
        }
    }

    /// Update the CO2 reading for color selection
    pub fn set_co2(&mut self, co2_ppm: u16) {
        self.current_co2 = co2_ppm;
    }

    /// Set maximum LED brightness based on ambient light sensor reading.
    ///
    /// Pass the raw 12-bit ADC value from the LDR photoresistor.
    pub fn set_ambient_light(&mut self, adc_value: u16) {
        self.max_brightness = adc_to_max_brightness(adc_value);
    }

    /// Set button state for LED color override.
    pub fn set_button_state(&mut self, state: ButtonState) {
        self.override_color = match state {
            ButtonState::Released => None,
            ButtonState::Pressed => Some(colors::PURPLE),
            ButtonState::LongPressed => Some(colors::WHITE),
        };
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

        let base_color = self.override_color.unwrap_or_else(|| co2_to_color(self.current_co2));
        let mut led_colors = [RGB8::default(); NUM_LEDS];

        // Animation parameters
        // With gamma correction (γ=2.0), brightness is perceptually linear
        // so we can use a wider range for more visible animation
        const WAVE_SPEED: f32 = 2.0; // Slow, gentle pulse
        const BASE_MIN: f32 = 0.1;
        const BASE_MAX: f32 = 1.0;

        // Scale brightness range by ambient light level
        let effective_min = BASE_MIN * self.max_brightness;
        let effective_max = BASE_MAX * self.max_brightness;

        for i in 0..NUM_LEDS {
            let brightness = pulse_brightness(
                i,
                self.time_counter,
                WAVE_SPEED,
                effective_min,
                effective_max,
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
