# SCD40 Air Quality Monitor

CO2 air quality monitoring using the SCD40 sensor with AHT20/BMP280 on a Raspberry Pi Pico W, built with Embassy async Rust.

Submits sensor data to a REST API over WiFi. WS2812B LED array displays CO2 levels with animated color feedback.

## Architecture

The firmware uses a multi-task architecture for reliability:

```
┌─────────────────────────────────────────────────┐
│ Main Task (LED + Sensors)                       │
│  - Updates LEDs every 50ms (never blocks)       │
│  - Reads sensors every 5s (with timeout)        │
│  - Sends readings via channel (non-blocking)    │
│  - Feeds watchdog every iteration               │
└─────────────────────┬───────────────────────────┘
                      │ Channel (non-blocking)
┌─────────────────────▼───────────────────────────┐
│ API Submit Task (Network)                       │
│  - Receives readings from channel               │
│  - Handles TLS/HTTP independently               │
│  - Runs without blocking main loop              │
└─────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────┐
│ Hardware Watchdog (8s timeout)                  │
│  - Resets device if main loop hangs             │
│  - Protects against I2C lockups                 │
└─────────────────────────────────────────────────┘
```

## Hardware

- **Board**: Raspberry Pi Pico W
- **CO2 Sensor**: SCD40 (I2C address 0x62) - True NDIR CO2 measurement
- **Temp/Humidity**: AHT20 (I2C address 0x38)
- **Pressure**: BMP280 (I2C address 0x77)
- **LEDs**: WS2812B array (8 LEDs on GPIO16)

## Pin Connections

| Pico W Pin | GPIO  | Function      | Connection     |
|------------|-------|---------------|----------------|
| Pin 6      | GP4   | I2C SDA       | Sensor SDA     |
| Pin 7      | GP5   | I2C SCL       | Sensor SCL     |
| Pin 21     | GP16  | WS2812B Data  | LED Data In    |
| Pin 36     | 3V3   | Power         | Sensors 3.3V   |
| Pin 38     | GND   | Ground        | Common GND     |

## Configuration

Create a `.env` file in the project root with your WiFi and API credentials:

```env
WIFI_SSID=YourNetworkName
WIFI_PASSWORD=YourPassword
API_ENDPOINT=https://your-api.example.com/sensor-data
API_KEY=your-api-key
```

These values are compiled into the firmware at build time.

## Building

Requires a debug probe (e.g., Raspberry Pi Debug Probe, Picoprobe) for flashing and logging.

```bash
# Install probe-rs
cargo install probe-rs-tools

# Build release
cargo build --release

# Flash and run with RTT logging
cargo run --release
```

## Feature Flags

| Flag | Description |
|------|-------------|
| `scd40-frc` | Perform forced recalibration of SCD40 on startup. Expose sensor to fresh outdoor air (~428 ppm) before use. |

```bash
# Build with forced recalibration
cargo build --release --features scd40-frc
```

## Viewing Logs

Logs are output via RTT (Real-Time Transfer) and require a debug probe:

```bash
# View logs from running device
probe-rs attach --chip RP2040

# Or run with automatic log display
cargo run --release
```

The RTT buffer persists in RAM, so you can connect a debug probe to a running device to read recent logs - useful for debugging issues that occur after hours/days of operation.

## Sensor Readings

After a 60-second warmup period, the device outputs readings every 5 seconds:
- **CO2**: Carbon dioxide concentration in ppm (SCD40)
- **Temperature**: In Celsius and Fahrenheit (AHT20)
- **Humidity**: Relative humidity percentage (AHT20)
- **Pressure**: Atmospheric pressure in hPa (BMP280)

Data is submitted to the configured API endpoint every 2 minutes.

## LED Behavior

The WS2812B LED array displays CO2 levels with a pulsing wave animation:

| CO2 Level      | Color  | Air Quality |
|----------------|--------|-------------|
| < 450 ppm      | Green  | Excellent   |
| 450-600 ppm    | Blue   | Good        |
| 600-900 ppm    | Amber  | Moderate    |
| > 900 ppm      | Red    | Poor        |

The onboard Pico W LED (controlled via CYW43) blinks briefly during sensor reads.

## Reliability Features

- **Hardware Watchdog**: 8-second timeout resets the device if the main loop hangs
- **I2C Bus Recovery**: Toggles SCL at startup to clear stuck I2C slaves
- **Sensor Timeouts**: 5-second timeout on SCD40 reads prevents indefinite blocking
- **Separate Network Task**: API submissions run independently, never blocking LED animation
- **Health Logging**: Uptime logged every 5 minutes for debugging long-running issues
- **Automatic Reset**: After 5 consecutive I2C failures, watchdog triggers a reset

## Project Structure

```
src/
├── main.rs         # Entry point, task orchestration, watchdog
├── leds.rs         # WS2812B LED control with PIO
├── network.rs      # WiFi and HTTPS API submission
├── sensors/
│   ├── mod.rs      # Sensor module exports
│   ├── scd40.rs    # SCD40 CO2 sensor driver
│   ├── aht20.rs    # AHT20 temp/humidity driver
│   └── bmp280.rs   # BMP280 pressure driver
└── types.rs        # Shared data types
```
