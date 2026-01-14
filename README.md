# ENS160 Air Quality Sensor

Air quality monitoring using the ENS160 sensor on a Raspberry Pi Pico W, built with Embassy async Rust.

## Hardware

- **Board**: Raspberry Pi Pico W
- **Sensor**: SparkFun Environmental Combo Breakout (ENS160/BME280)
  - [Product Page](https://www.sparkfun.com/products/22858)

## Pin Connections

| Pico W Pin | GPIO | Function | Sensor Pin |
|------------|------|----------|------------|
| Pin 6      | GP4  | I2C SDA  | SDA        |
| Pin 7      | GP5  | I2C SCL  | SCL        |
| Pin 9      | GP6  | INT      | INT        |
| Pin 36     | 3V3  | Power    | 3.3V       |
| Pin 38     | GND  | Ground   | GND        |

## Sensor Address

The SparkFun board uses I2C address `0x53` (ADDR jumper set high). If your board uses `0x52`, change `Ens160::new_secondary_address()` to `Ens160::new()` in `src/main.rs`.

## Building

Currently configured to build and flash over USB. Future version will be updated to use `probe-rs` instead.

```bash
# Install required tools
cargo install elf2uf2-rs

# Build
cargo build --release

# Build and flash (hold BOOTSEL, plug in USB, release)
cargo run --release
```

## Viewing Output

Logs can be viewed by default with a debug probe. An optional feature flag allows USB debugging.

```bash
# Build and flash with USB debugging enabled
cargo run --release --features usb-logging
```

Connect to USB serial to see sensor readings:

```bash
# macOS
cat /dev/tty.usbmodem*

# Linux
cat /dev/ttyACM0
```

## Sensor Readings

After a 3-minute warmup period, the sensor outputs every 5 seconds:
- **AQI**: Air Quality Index (1-5)
- **eCO2**: Equivalent CO2 in ppm
- **TVOC**: Total Volatile Organic Compounds in ppb

## LED Behavior

- **Slow blink (200ms on, 800ms off)**: Warmup period
- **Brief flash**: Reading sensor
- **Rapid blink**: Error state

LED code is specific to the Pico W LED, which is controlled via the wireless chip rather than a GPIO pin.

## License

MIT
