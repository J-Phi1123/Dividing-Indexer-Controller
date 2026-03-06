# Dividing Indexer Controller

ESP32 firmware for:
- Web server stepper control
- Two digital input buttons
- SSD1306 I2C display status output
- AP-mode network provisioning page with encrypted storage
- RGB status LED state indication (LED #1)
- Mobile-friendly UI with live indexer dial

## Hardware Mapping (current defaults)
- Stepper (Maker ESP32 `Stepper1`): `GPIO27`, `GPIO13`, `GPIO2`, `GPIO4`
- Button 1: `GPIO32` (active-low, uses internal pull-up)
- Button 2: `GPIO33` (active-low, uses internal pull-up)
- OLED I2C: `SDA=GPIO4`, `SCL=GPIO5`, address `0x3C`
- RGB LEDs: `GPIO16` (4 LEDs total; first LED used for status)

Notes:
- If you switch firmware to `Stepper2`, set the board switch for `M3/M4` to `Motor` mode.
- User request said "SSD1036"; this project is configured for `SSD1306` (common 128x64 I2C OLED).

## Build / Flash (PlatformIO)
1. Install PlatformIO CLI.
2. Set Wi-Fi credentials in `src/main.cpp`:
   - `WIFI_SSID`
   - `WIFI_PASS`
3. Build:
   - `pio run`
4. Flash:
   - `pio run -t upload`
5. Monitor serial:
   - `pio device monitor`

## Web Endpoints
- `GET /` browser UI
- `GET /status` JSON status
- `POST /stepper/enable`
- `POST /stepper/disable`
- `POST /stepper/stop`
- `POST /stepper/move?steps=200&dir=1`
- `POST /stepper/speed?value=400`
- `POST /stepper/accel?value=120`
- `POST /indexer/step?dir=1`
- `POST /indexer/set_gears?value=40`
- `POST /config/network` (AP mode only)

## Indexer Math
- Mechanical relationship: stepper rotates `360°` x `800` for indexer `360°`.
- `steps_per_indexer_rev = 200 * 20 * 40 = 160000`
- `ticks_per_gear = round((200 * 20 * 40) / number_of_gears)`

## Stepper Drive Backend
- Motor movement now uses local `src/Stepper.cpp` / `src/Stepper.h` from the maker `stepperTest` project.
- `speed` is active.
- `accel` is accepted by the UI/API for compatibility but the Stepper backend does not implement acceleration ramps.

## AP Mode Network Provisioning
When STA connection fails, device starts AP mode and the webpage shows:
- `SSID`
- `Password`
- `Static IP`
- `Gateway`
- `Netmask`

Submitting this form stores config encrypted on-device (AES-256-CBC in NVS) and reboots.
If saved credentials exist, boot prioritizes them for STA connection and captures the boot-selected IP.

Security note:
- The AES key is currently hardcoded in `src/main.cpp` (`AES_KEY`).
- Change this key before production deployment.

## Wi-Fi Status LED (first RGB LED)
- Red: attempting STA connection to configured AP
- Green: STA connected successfully
- Blue: running in AP mode

## Button Actions
- Button 1 press: move `+200` steps
- Button 2 press: move `-200` steps

Adjust behavior in `handleButtonActions()` in `src/main.cpp`.
