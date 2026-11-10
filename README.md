# Esp32-gps-nmea-emulator
ESP32 GPS emulator for ArduPilot/PX4 bench testing: streams NMEA over UART with a map-based web UI, smooth simulated travel and a GPS power-load profile.


# ESP32 GPS Bench Emulator

An ESP32 that pretends to be a GPS module (NMEA 0183 over UART) for **ArduPilot / PX4 flight controllers**, with a web page for picking the position on a map. It also drives a PWM-controlled dummy load that mimics a GPS module's power-draw profile.

## Purpose

Test a flight controller's GPS port, GPS driver configuration and EKF behaviour **without a real GPS module or a clear sky view**. Useful when you need to tell a firmware or sketch bug apart from a flight-controller or wiring fault.

## Features

- NMEA output at 5 Hz, 115200 baud: `GGA`, `RMC`, `VTG`, `GSA`
- Web UI with a map (click or drag a pin, or search a place name)
- **Smooth travel** to the target at a configurable speed. Speed and course in `RMC`/`VTG` match the motion, so the EKF does not raise GPS glitch warnings. Set the speed to 0 for an instant jump
- Adjustable altitude, satellite count, HDOP, 3D fix on/off and fix delay
- Real UTC time and date from NTP (falls back to uptime when offline)
- PWM load output with an automatic GPS boot power profile or manual duty cycle
- Falls back to its own Wi-Fi access point if the lab network is unreachable

## Hardware

- ESP32 dev board (ESP32 WROOM)
- Flight controller with a free UART configured as GPS
- Optional: logic-level MOSFET plus a 22 ohm load resistor on GPIO25 for the power-draw profile

| ESP32 | Connects to |
|---|---|
| GPIO17 (TX) | FC GPS **RX** |
| GPIO16 (RX) | FC GPS **TX** |
| GPIO25 | Load MOSFET gate (optional) |
| GND | FC GND |

TX and RX must cross over (ESP32 TX goes to FC RX). Swapped wires are the most common reason the FC does not detect the GPS.

Note: on ESP32 WROVER modules GPIO16/17 are used by PSRAM, so change `GPS_UART_TX_PIN` / `GPS_UART_RX_PIN` in `main.cpp`.

## Build and flash

This is a [PlatformIO](https://platformio.org/) project (VS Code extension or CLI).

1. Clone the repo and open it in PlatformIO.
2. Open `src/main.cpp` and set your Wi-Fi credentials:
   ```cpp
   const char* ssid = "xxxxxxxx";
   const char* pwd  = "xxxxxxxx";
   ```
3. Build and upload (`pio run -t upload`), then open the serial monitor at 115200 baud.

The serial monitor prints the IP address and a `[DBG]` line every second with the latest `GPGGA` sentence.

## Usage

### 1. Open the web UI

- On your Wi-Fi: `http://gpsbench.local` or the IP shown in the serial monitor
- If Wi-Fi fails, join the access point `GPS-Bench` (password `gpsbench123`) and browse to `http://192.168.4.1`

### 2. Configure the flight controller (ArduPilot)

Set the parameters below, write them, then **reboot the FC**. Parameter names vary slightly between versions: older firmware uses `GPS_TYPE` / `GPS_RATE_MS` instead of `GPS1_TYPE` / `GPS1_RATE_MS`.

| Parameter | Value | Note |
|---|---|---|
| `SERIALn_PROTOCOL` | 5 (GPS) | `n` is the port you wired to (check your board's UART-to-SERIAL mapping) |
| `SERIALn_BAUD` | 115 | 115200 |
| `GPS1_TYPE` | 5 | NMEA |
| `GPS2_TYPE` | 0 | No second GPS |
| `GPS_AUTO_CONFIG` | 0 | The emulator cannot answer config commands |
| `GPS1_RATE_MS` | 200 | Matches the 5 Hz output |

### 3. Check it in Mission Planner

- The Messages tab shows `GPS 1: detected NMEA`
- The HUD shows **3D Fix** with 12 satellites and HDOP of about 0.9
- Press `Ctrl+F` and open MAVLink Inspector, then `GPS_RAW_INT`, to see the raw fix, lat/lon and satellite count

### 4. Move the position

Click the map or drag the pin to set a target. The red dot is where the FC currently thinks it is, and it travels to the pin at the configured speed.

| Control | Description |
|---|---|
| Lat / Lon | Target position (also settable by typing) |
| Travel speed (m/s) | Speed toward the target. 0 jumps instantly (useful to provoke a GPS glitch on purpose). Raise it for far-away targets |
| Alt, Satellites, HDOP | Values reported in the NMEA sentences |
| Provide 3D fix / Fix delay | Disable the fix, or delay it by N seconds after boot or Restart profile |
| Auto GPS boot profile | Load PWM follows an acquisition, tracking, steady-state profile (about 15 s, 15 s, then steady) |
| Manual PWM | Fixed 0-255 duty cycle when auto profile is off |

### HTTP API

| Endpoint | Description |
|---|---|
| `/api/status` | Current state as JSON |
| `/api/set?lat=..&lon=..&speed=..&alt=..&sats=..&hdop=..&fix=1&fixdelay=..&auto=1&pwm=..` | Set any subset of values |
| `/api/restart` | Restart the boot profile and fix delay timer |

## Troubleshooting

- **FC never detects the GPS**: check TX/RX are crossed, the `SERIALn` port matches the wiring, and `GPS1_TYPE` is 5. Confirm the sketch is streaming by viewing the pin with a USB-TTL adapter at 115200 baud
- **"EKF3 waiting for GPS config data"**: usually `GPS2_TYPE` is non-zero, or no GPS was detected
- **GPS glitch warnings after a far jump**: lower the jump by using a non-zero travel speed
- **Map does not load**: the browser needs internet for map tiles; the lat/lon fields still work offline

## Safety

This is a bench-testing tool. Do not use it as a GPS source on a vehicle that will fly.

## License

Add a license of your choice (for example MIT) before publishing.
