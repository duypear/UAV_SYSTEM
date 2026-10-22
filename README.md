# UAV System

This project consists of a flight controller running on an STM32F103 and a ground station running on an ESP32-C3. The flight controller reads sensors and receiver inputs, controls four ESCs, and transmits telemetry wirelessly to the ground station. The ground station displays the telemetry on a web dashboard over Wi-Fi.

> **Safety warning:** This system can control motors and a payload-release mechanism. Remove all propellers before powering on, flashing firmware, or testing controls for the first time. Verify wiring, power, and failsafe behavior on the bench before flight. This README does not replace a complete safety and airworthiness check.

## Components

| Directory | Hardware / technology | Role |
| --- | --- | --- |
| [`uav_fc/`](./uav_fc/) | STM32F103C8T6 (Blue Pill), STM32 HAL, STM32CubeIDE | Reads the IMU and RC receiver; stabilizes the aircraft; outputs PWM to four ESCs and a servo; transmits telemetry. |
| [`uav_ground_station/`](./uav_ground_station/) | ESP32-C3, ESP-IDF, nRF24L01, Wi-Fi SoftAP, and HTTP server | Receives and decodes telemetry, monitors link quality, and serves the web dashboard. |

## Data flow

```text
RC receiver ─┐
GY-87       ─┤
GPS/BMP388* ─┤──> STM32F103 Flight Controller
             │       ├── PWM ──> 4 ESCs / motors
             │       ├── PWM ──> payload-release servo
             │       └── nRF24L01 ──> ESP32-C3 Ground Station
             │                              └── Wi-Fi SoftAP / web dashboard
```

`*` GPS and BMP388 support is present in the firmware; actual operation and display depend on the sensors, wiring, and configuration in use. Altitude-hold mode is not enabled in the current firmware.

## Telemetry protocol

Both firmware projects use the shared protocol definitions in `uav_fc/Core/Inc/telemetry_protocol.h` and `uav_ground_station/main/telemetry_protocol.h`.

- Fixed 32-byte packets, little-endian, with an additional CRC-8 check.
- nRF24L01 channel 80 (2480 MHz), at 1 Mbps.
- Frame types: attitude (`0xA1`), RC (`0xA2`), status (`0xA3`), tuning/navigation (`0xA4`), configuration (`0xA5`), and GPS (`0xA6`).
- When changing the protocol, update both definitions and keep the transmitter and receiver compatible.

## Development requirements

- **Flight Controller:** STM32CubeIDE and the STM32 ARM GCC toolchain; flash/debug over ST-LINK SWD or UART as configured.
- **Ground Station:** ESP-IDF (the project documents compatibility with ESP-IDF v5.x/v6.x), CMake, and an ESP32-C3 board.
- **Radio hardware:** nRF24L01 modules on both ends; both must use the same channel, address, and protocol.

## Build and flash

### Flight Controller (STM32F103)

1. Open `uav_fc/` in STM32CubeIDE.
2. Build the `uav_complete_fc` project.
3. Flash the firmware to the STM32F103C8T6 board using ST-LINK/SWD or the project's configured debug setup.
4. To view logs, connect a USB-to-TTL adapter as described in the setup guide and open a serial monitor at 115200 baud.

### Ground Station (ESP32-C3)

Open a terminal with the ESP-IDF environment configured, then run:

```bash
cd uav_ground_station
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

Replace `COMx` with the board's COM port. If the target is already configured correctly, you can skip `set-target`.

## Hardware connections

### ESP32-C3 to nRF24L01

| nRF24L01 | ESP32-C3 |
| --- | --- |
| GND | GND |
| VCC | 3.3 V |
| CE | GPIO 3 |
| CSN | GPIO 7 |
| SCK | GPIO 4 |
| MOSI | GPIO 6 |
| MISO | GPIO 5 |
| IRQ | Leave unconnected |

Power the nRF24L01 from 3.3 V; do not connect it to 5 V. Place a 10-100 uF capacitor close to the radio module to reduce supply dips, especially when using a PA/LNA module.

### Flight Controller

See [`uav_fc/FLIGHT_SETUP.md`](./uav_fc/FLIGHT_SETUP.md) for detailed pin assignments, power, receiver, GY-87, ESC, servo, and GPS wiring, plus the bring-up procedure. The Ground Station and dashboard guide is in [`uav_ground_station/README.md`](./uav_ground_station/README.md).

## Using the dashboard

1. Power on the ESP32-C3 Ground Station.
2. Connect a phone or computer to its Wi-Fi SoftAP:
   - SSID: `UAV-GroundStation`
   - Password: `uavtelemetry`
3. Open `http://192.168.4.1` in a browser.
4. Once the Flight Controller and radio link are active, the dashboard displays link status, attitude, RC inputs, motor outputs, GPS, and other transmitted telemetry.

## Testing notes

- Verify the nRF24L01 has a stable 3.3 V supply and shares ground with the ESP32-C3.
- Confirm that both firmware projects use matching radio channel, address, and protocol settings.
- Test failsafe, arming/disarming, motor direction, and servo behavior with propellers removed.
- Refer to [`uav_fc/FLIGHT_SETUP.md`](./uav_fc/FLIGHT_SETUP.md) for the bring-up checklist and safety conditions for the current firmware.
