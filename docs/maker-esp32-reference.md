# Maker ESP32 Reference (for Web Server + Stepper Development)

## Purpose
This document captures hardware details from `nulllaborg/maker-esp32` needed to build firmware that exposes a web server and controls a stepper motor.

## Board / SoC Specs
- Module: `ESP32-WROOM-32E`
- Wireless: 2.4 GHz Wi-Fi + Bluetooth (dual mode)
- Memory: `448KB ROM`, `520KB SRAM`, `4MB Flash`
- USB-UART: `CH341`
- Onboard motor driver: `TB67H450FNG`
- Motor driver max current (repo-listed): `3.5A` single motor
- Motor supply input: external `DC 6V-16V` via `5.5 x 2.1mm` jack
- Board dimensions / mass: `80mm x 57mm`, `35g`

## Pinout Mapping

### DC Motor Ports
- `M1`: `GPIO27`, `GPIO13`
- `M2`: `GPIO4`, `GPIO2`
- `M3`: `GPIO17`, `GPIO12`
- `M4`: `GPIO14`, `GPIO15`

### Stepper Ports (shared with motor channels)
- `Stepper1`: `GPIO27`, `GPIO13`, `GPIO2`, `GPIO4` (shares `M1` + `M2`)
- `Stepper2`: `GPIO17`, `GPIO12`, `GPIO14`, `GPIO15` (shares `M3` + `M4`)

### Other Interfaces
- Servo outputs: `GPIO25`, `GPIO26`, `GPIO32`, `GPIO33`
- SPI header: `GPIO5`, `GPIO18`, `GPIO19`, `GPIO23`
- General IO exposed: `GPIO12`, `GPIO14`, `GPIO15`, `GPIO17`, `GPIO34`, `GPIO35`, `GPIO36`, `GPIO39`
- RGB LED: `GPIO16`
- OLED: dedicated I2C interface (README does not explicitly list SDA/SCL numbers)

## Hardware Constraints
- `Stepper1` cannot be used independently of `M1/M2` pin ownership; they are physically shared.
- `Stepper2` cannot be used independently of `M3/M4` pin ownership; they are physically shared.
- For `M3/M4` (and thus `Stepper2`) operation, board switch mode matters:
  - `IO` position: pins act as GPIO
  - `Motor` position: pins route to motor driver
- `GPIO34`, `GPIO35`, `GPIO36`, `GPIO39` are input-only and do not provide internal pull-up/pull-down.
- External motor power on DC jack is required for motor/stepper operation.

## Firmware Notes for Web + Stepper
- Use ESP32 Arduino core `>= 3.0.0` (repo examples indicate this baseline).
- Implement non-blocking stepper stepping (timer or loop scheduler) so HTTP server remains responsive.
- Suggested HTTP endpoints:
  - `POST /stepper/enable`
  - `POST /stepper/disable`
  - `POST /stepper/move` (direction + steps)
  - `POST /stepper/speed` (steps/sec or RPM abstraction)
  - `GET /stepper/status`
- Validate coil order before production: exact terminal order for stepper connector should be verified against the board schematic and/or vendor stepper test example.

## Sources
- Repo: <https://github.com/nulllaborg/maker-esp32>
- README: <https://raw.githubusercontent.com/nulllaborg/maker-esp32/master/README.md>
