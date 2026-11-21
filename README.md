# Fish shredder and grinder

Firmware for one weight-triggered fish batch on an ESP32-S3. The operator loads fish. When the weight is at or above the target and stable, the controller starts the grinder, then the shredder, opens the weighing-bin door, closes it, runs the motors on, and stops.

Motor power, overload protection, and the emergency stop stay in the hardwired electrical system. The firmware commands three outputs. It does not sense door position, and it does not resume a batch after reset.

The behavior contract is [docs/superpowers/specs/2026-11-21-fish-shredder-grinder-firmware-design.md](docs/superpowers/specs/2026-11-21-fish-shredder-grinder-firmware-design.md).

## What you need

- ESP-IDF 6.1 (`>=6.1.0,<6.2.0`). This tree was built with 6.1.0. Stay on build system v1. ESP-IDF 6.1.1 build system v2 fails the bootloader when octal PSRAM is enabled (IDFGH-18754). The root `CMakeLists.txt` keeps the classic project form for that reason.
- A host C compiler, CMake, and Ninja, for the tests that do not need a board. On the development machine those are GCC 15.2.0, the CMake and Ninja shipped with ESP-IDF, and a PowerShell ESP-IDF environment.
- The module is an ESP32-S3 N16R8: 16 MB flash and octal PSRAM. The panel is an ILI9341, 320 by 240, driven by LVGL 9.5.0 and `espressif/esp_lcd_ili9341` 2.1.0.

No Arduino framework and no Arduino libraries.

## Host tests

From the repository root, in a shell where `cmake`, `ninja`, and `gcc` are on `PATH`:

```powershell
cmake -S test/host -B build/host -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

A good run ends with `100% tests passed`. The seven programs cover CRC-16/MODBUS, the settings record, the batch state machine, the Modbus master, scale decode and the compiled profile, the encoder, and screen navigation. They compile plain C and do not include ESP-IDF headers.

## Build and flash

Open an ESP-IDF 6.1 shell, then from the repository root:

```powershell
idf.py -B build/idf set-target esp32s3
idf.py -B build/idf build
idf.py -B build/idf -p COM5 flash monitor
```

Replace `COM5` with the board's serial port. `set-target` is only needed once. It writes `sdkconfig`. Commit `sdkconfig.defaults`, not `sdkconfig`.

Use `-B build/idf` when `build/host` also exists. `idf.py set-target` refuses to wipe a `build/` directory that is not its own CMake tree, and the host tests live at `build/host`.

`idf.py build` prints `Project build complete` when the image links. The image is about 575 KB in a 3 MB factory partition. Live scale, contactor, actuator, and fish checks are commissioning steps and are not part of the host suite.

## Operator panel

The encoder is active low. One detent moves a selection or changes the armed setting by one step. A short press activates the selection. A long press on the main screen opens the menu while idle, and opens stop confirmation during a batch. The switch level must stay the same for 20 ms before it counts, so a noise spike is not a press. The long-press time is 800 ms after that stable press is accepted.

Settings and manual mode open only from idle. Saving settings writes them to NVS and publishes them to the core. Back throws away the unsaved edit. A fault screen shows `I/O link lost` until the operator acknowledges it, and acknowledgement is accepted only after a later I/O transaction has succeeded.

Weights on the panel are kilograms with two decimals. `25000` grams is `25.00 kg`.

## Changing the machine

Chip options live in `sdkconfig.defaults`: 16 MB flash, octal PSRAM, and a 1000 Hz tick. Slave IDs, baud, parity, pins, and the scale map live in `main/device/profile.c`. Either change needs a rebuild and flash. The panel edits weights and timers only.

Initial profile:

| Bus | Value |
|---|---|
| UART | UART 1, TX GPIO 17, RX GPIO 18, DE GPIO 16, 9600 8N1, RS485 half duplex |
| Scale | Slave 1, function `0x03`, register 0, count 2, signed 32-bit big-endian grams |
| Outputs | Slave 2, function `0x0F`, coil 0, count 3. Bit 0 shredder, bit 1 grinder, bit 2 door open |
| Panel | ILI9341 on SPI2, SCLK 4, MOSI 5, MISO 6, CS 7, DC 8, RST 9, backlight 10 active high, 20 MHz, `swap_xy` on |
| Encoder | A GPIO 11, B GPIO 12, switch GPIO 13, active low, pull-ups |

The decoder treats a low switch as pressed. Quadrature debounce (2 ms), steps per detent (4), button debounce (20 ms), and long press (800 ms) are compiled into `main/ui/encoder_decode.c`. The same numbers are stored on the profile for the record. Editing only `profile.timing` does not change the decoder.

NVS namespace is `machine`, key `cfg`. The record is 54 bytes: version 1, twelve little-endian fields, then CRC-16/MODBUS of the first 52 bytes. A failed load keeps the defaults and raises `settings_reset`, including the first boot when no record exists.

## Layout

| Path | Role |
|---|---|
| `main/machine/` | State machine and the 10 ms control task |
| `main/modbus/` | RTU master and the scale poll / coil writer |
| `main/settings/` | Settings codec and NVS store |
| `main/status/` | Mutex snapshots and the operator command queue |
| `main/ui/` | Encoder decoder and screen logic, host-tested |
| `main/hmi/` | ILI9341, encoder GPIO, and the LVGL task |
| `main/device/` | Compiled profile, scale decode, and the UART board |
| `test/host/` | Host test programs |
| `docs/superpowers/specs/` | Firmware contract |
| `docs/superpowers/plans/` | Implementation plan this tree was built from |

Three tasks run after boot: HMI (priority 5, CPU 0), Modbus (priority 9, CPU 1), then control (priority 10, CPU 1). Control decides the outputs. Modbus is the only owner of the RS485 UART. The HMI task is the only caller of `lv_timer_handler`.
