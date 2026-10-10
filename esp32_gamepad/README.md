# esp32_gamepad

Bluetooth gamepad receiver for the grblHAL gamepad plugin of this machine. An ESP32-WROOM-32
connects a PS4 controller (DualShock 4; DualSense and Xbox Series X|S work as well) with
[Bluepad32](https://github.com/ricardoquesada/bluepad32) and forwards its state over UART to
the BTT Octopus Pro. Protocol, allow list and wiring are described at the top of
[`sketch.cpp`](sketch.cpp).

## Build

Bluepad32 is built from its official ESP-IDF + Arduino template, only `sketch.cpp` is ours:

ESP-IDF does not build in paths with spaces, so the template lives outside this repository
(here `F:\VisualCode\esp32_gamepad_build`):

1. Clone the template with its components to a path without spaces:
   ```
   git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template.git esp32_gamepad_build
   ```
2. Adapt the template to PlatformIO Core 6.2 (the template's platform 54.03.21 needs SCons 4.8 and fails with
   `No module named 'SCons.Tool.FortranCommon'`):
   - `platformio.ini`: `platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.312-1/platform-espressif32.zip`
   - Arduino component to the matching version: `git -C components/arduino fetch --depth 1 origin tag 3.3.12` and `git -C components/arduino checkout 3.3.12`
   - `components/cmd_system/cmd_system.c`: add `#include <driver/gpio.h>` (needed with ESP-IDF 5.5)
3. Copy `esp32_gamepad/sketch.cpp` to `esp32_gamepad_build/main/sketch.cpp`.
4. Build and flash the environment `esp32dev` with PlatformIO from PowerShell or VS Code, not from Git Bash
   (ESP-IDF refuses to install its tools in an MSYS shell).

## Wiring

| Octopus TFT header | ESP32 DevKit |
|---|---|
| 5V | 5V / VIN |
| GND | GND |
| TX (PA9) | GPIO16 (RX2) |
| RX (PA10) | GPIO17 (TX2) |

## Pairing

- Hold **BOOT** for 3 s: the on-board LED blinks for 60 s, put the controller in pairing mode
  (PS4: Share + PS until the light bar flashes). The controller is added to the allow list.
- A controller that is already allowed can pair again at any time.
- Hold **BOOT** for 10 s to clear the allow list.
