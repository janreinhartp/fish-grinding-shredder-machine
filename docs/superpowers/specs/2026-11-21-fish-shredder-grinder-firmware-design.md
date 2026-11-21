# Fish Shredder and Grinder Firmware Design

Date: 2026-11-21

The firmware runs an automatic, weight-triggered batch on an ESP32-S3. The operator loads fish. When the weight is at or above the target and stable, the controller starts the grinder, then the shredder, opens the weighing-bin door, closes it, runs the motors on, and stops. Motor power, overload protection, and emergency stop stay in the hardwired electrical system. This document is the firmware contract.

## Locked decisions

- The deliverable is firmware that builds and flashes with ESP-IDF v6.1.1. The scale register map, baud settings, and panel controller live in a compiled device profile.
- An I/O transaction failure commands shredder off, grinder off, and door close, then latches `MACHINE_FAULT` until the operator acknowledges. Acknowledgement is accepted only after a later I/O transaction has succeeded.
- Scale loss before a batch is latched keeps the machine waiting. Scale loss after the latch finishes the timing sequence, raises a sticky scale warning, and blocks the next batch until the scale is healthy and the weight is below the reset threshold and stable.
- Software stop is a confirmed encoder action. It commands all three outputs off, cancels the batch, returns to `MACHINE_IDLE`, and requires the normal re-arm. Settings and manual mode open only from `MACHINE_IDLE`.
- Chip options (flash, PSRAM, tick rate) live in `sdkconfig.defaults`. Slave IDs, baud, parity, pins, and the scale map live in `main/device/profile.c`. Changing either one requires a rebuild and flash. The TFT edits weights and timers only.
- Stability is the peak-to-peak spread over the stability window. The spread must be within the tolerance and the latest sample must still be at or above the target. If the spread exceeds the tolerance, the window starts over at the current sample.
- The sequence, stability window, re-arm rule, and output requests live in plain C with an injected clock. A small Modbus RTU master owns the UART. Three FreeRTOS tasks exist: control, Modbus, and HMI.

## Global constraints

- Framework: native ESP-IDF v6.1.1. Target `esp32s3`. Language C. Build system CMake.
- No Arduino framework and no Arduino libraries.
- RTOS: FreeRTOS. Tick rate 1000 Hz.
- Controller module: ESP32-S3 N16R8 (16 MB flash, octal PSRAM).
- Display library: LVGL 9.5.0. Panel driver: `espressif/esp_lcd_ili9341` 2.1.0. The HMI task is the only caller of `lv_timer_handler`. `esp_lvgl_port` is not used.
- Root `CMakeLists.txt` uses the classic project form (`include($ENV{IDF_PATH}/tools/cmake/project.cmake)` then `project(...)`). ESP-IDF v6.1.1 build system v2 fails the bootloader when `CONFIG_SPIRAM=y` (IDFGH-18754). This project stays on build system v1 so octal PSRAM can be enabled.
- Weights inside the machine core are integer grams. `25000` is 25.00 kg. Tolerance `100` is 0.10 kg.
- Time inside the core is a caller-supplied `uint32_t` millisecond count. Elapsed time is unsigned subtraction.
- The core performs at most one state transition per `machine_core_step` call.
- Door command off means close. The firmware does not claim the actuator has reached a position.
- Safety outputs on boot, stop, fault, and leaving manual: shredder off, grinder off, door close.
- A batch in progress is never resumed after reset. Boot starts in `MACHINE_IDLE`, disarmed, outputs off.
- One operator command is applied per step.
- Host tests compile the plain C modules with the system C compiler. They do not include ESP-IDF headers.

## Architecture

| Unit | Responsibility | Depends on |
|---|---|---|
| `machine_core` | Stability window, arming, state machine, fault latch, desired outputs | Types and settings only |
| `control_task` | Call the core every 10 ms and publish status | Core, status bus, device profile |
| `modbus_master` | One RTU transaction: build the frame, check CRC, classify the reply | `crc16`, a byte transport |
| `modbus_task` | The only owner of the RS485 UART. Poll the scale. Write the three outputs | Master, profile, board transport, status bus |
| `settings_codec` | Versioned little-endian record and CRC | `crc16`, settings struct |
| `settings_store` | Load and save that record in NVS | Codec, NVS |
| `encoder_decode` | Quadrature detents, short press, long press | None |
| `ui_logic` | Screen navigation and operator commands | Settings validation |
| `hmi_task` | LVGL render, encoder samples, NVS save | UI logic, display, encoder hardware, status bus |
| `device profile` | Compiled pins, bus, scale map, panel, periods | None |
| `status_bus` | Mutex copies and the command queue | FreeRTOS |

Control decides the output image. Modbus applies it. The HMI reads snapshots and sends commands. A running batch keeps the timer values copied at the latch even if settings are published again.

## Clock, periods, and task placement

All firmware timestamps are `(uint32_t)(esp_timer_get_time() / 1000)`.

| Item | Value |
|---|---|
| Control period | 10 ms |
| Scale poll | 100 ms |
| Output rewrite | 500 ms, and also on every image change |
| Scale stale limit | 500 ms |
| Modbus response timeout | 100 ms |
| Button debounce | 20 ms |
| Quadrature debounce | 2 ms |
| Long press | 800 ms |
| Quadrature transitions per detent | 4 |
| Control priority / CPU / stack | 10 / CPU 1 / 4096 |
| Modbus priority / CPU / stack | 9 / CPU 1 / 4096 |
| HMI priority / CPU / stack | 5 / CPU 0 / 8192 |
| Command queue depth | 8 |

Control is pinned above Modbus so a UART wait does not delay a state step. Tasks yield with `vTaskDelayUntil` or `vTaskDelay`. No task calls `vTaskDelay` inside the machine core.

## Output image

```c
typedef struct {
    bool shredder;   /* true commands DO1 on */
    bool grinder;    /* true commands DO2 on */
    bool door_open;  /* true commands DO3 on, which requests open */
} output_image_t;
```

The Modbus task writes this image when it changes and again every 500 ms. A successful UART transmission is not success. Success is a valid Modbus reply. Until the first I/O attempt completes, `io.link_ok` stays true and `io.attempted` stays false so boot does not raise a fault. The first failed I/O transaction sets `link_ok` false. The next successful transaction sets it true.

## Weight rules

A scale sample presented to the core is:

```c
typedef struct {
    int32_t grams;
    bool valid;
} scale_sample_t;
```

`control_task` sets `valid` only when the latest scale transaction succeeded, `grams >= 0`, and the sample age is at most 500 ms. A decoded negative weight stays in the snapshot for the display and is passed to the core with `valid == false`.

The stability window stores the minimum and maximum accepted sample and the start time.

- An invalid sample clears the window. `stable` becomes false.
- The first valid sample opens a window with min = max = that sample.
- A later valid sample updates min and max. If `max - min > tolerance`, the window restarts at that sample.
- `stable` is true when the window is open, `now - start >= stable_ms`, and `max - min <= tolerance`.

`batch_ready` is true when the sample is valid, `grams >= minimum_batch_g`, `grams >= target_g`, and `stable` is true.

`rearm_ready` is true when the sample is valid, `grams < reset_threshold_g`, and `stable` is true. Equality with the reset threshold does not re-arm.

On boot, and after stop, fault acknowledgement, or leaving manual, the batch is disarmed. While disarmed, `batch_ready` does not start a batch. The machine arms on `rearm_ready`. Arming clears the stability window so the climb up to the target starts a new window.

A latched batch runs once. Weight that remains above the target does not start another batch.

## Settings

Defaults and edit limits:

| Field | Default | Minimum | Maximum | Encoder step |
|---|---:|---:|---:|---:|
| `target_g` | 25000 | 100 | 50000 | 100 |
| `minimum_batch_g` | 1000 | 100 | 50000 | 100 |
| `tolerance_g` | 100 | 10 | 2000 | 10 |
| `stable_ms` | 2000 | 500 | 30000 | 100 |
| `reset_threshold_g` | 500 | 0 | 50000 | 100 |
| `grinder_startup_ms` | 2000 | 0 | 120000 | 100 |
| `shredder_startup_ms` | 2000 | 0 | 120000 | 100 |
| `door_open_ms` | 5000 | 0 | 120000 | 100 |
| `discharge_ms` | 10000 | 0 | 300000 | 100 |
| `door_close_ms` | 5000 | 0 | 120000 | 100 |
| `shredder_run_on_ms` | 5000 | 0 | 300000 | 100 |
| `grinder_run_on_ms` | 8000 | 0 | 300000 | 100 |

`machine_settings_valid` requires `target_g > 0`, `minimum_batch_g > 0`, `minimum_batch_g <= target_g`, `tolerance_g > 0`, `stable_ms > 0`, `reset_threshold_g >= 0`, and `reset_threshold_g < target_g`. The UI also keeps each field inside the table above. An edit that fails either check is discarded.

`machine_core_set_settings` rejects an invalid struct and keeps the previous one. At the latch into `MACHINE_WEIGHT_VALIDATION`, the core copies the current settings into a cycle record. Startup, door, discharge, and run-on timers for that batch come from the cycle record.

NVS namespace `machine`, key `cfg`. The codec writes 54 bytes, little-endian, in this order: `uint32 version` (value 1), then the twelve fields in the table order, then `uint16 crc16_modbus` of the first 52 bytes. Unpack fails when the length is not 54, the version is not 1, the CRC does not match, or the values are not valid. A failed load uses the defaults and raises `settings_reset`. A failed save keeps the previous core settings and raises `settings_save_failed`. A successful save clears both flags.

## State machine

States:

| State | Entry | Leave when |
|---|---|---|
| `MACHINE_IDLE` | Outputs off | Armed, `start_allowed`, and `batch_ready`: enter `MACHINE_WEIGHT_VALIDATION`. Disarmed and `rearm_ready`: set armed and clear the window |
| `MACHINE_WEIGHT_VALIDATION` | Copy cycle settings, disarm, store `trigger_weight_g`, outputs stay off | The next step enters `MACHINE_START_GRINDER` |
| `MACHINE_START_GRINDER` | Grinder on, shredder off, door close | `elapsed >= cycle.grinder_startup_ms` |
| `MACHINE_START_SHREDDER` | Shredder on, grinder stays on, door close | `elapsed >= cycle.shredder_startup_ms` |
| `MACHINE_OPEN_DOOR` | Door open, both motors on | `elapsed >= cycle.door_open_ms` |
| `MACHINE_DISCHARGING` | Door stays open, both motors on | `elapsed >= cycle.discharge_ms` |
| `MACHINE_CLOSE_DOOR` | Door close, both motors on | `elapsed >= cycle.door_close_ms` |
| `MACHINE_MOTOR_RUNON` | Door stays closed, both motors on | Shredder turns off when `elapsed >= cycle.shredder_run_on_ms`. Grinder turns off when `elapsed >= cycle.grinder_run_on_ms`. When both are off, enter `MACHINE_BATCH_COMPLETE` |
| `MACHINE_BATCH_COMPLETE` | Outputs off | `rearm_ready`: enter `MACHINE_IDLE` armed, clear the scale warning and the window |
| `MACHINE_MANUAL` | Outputs off, disarmed | Output commands update the image. Exit returns to disarmed `MACHINE_IDLE` with outputs off |
| `MACHINE_FAULT` | Outputs off, fault `FAULT_IO_LINK` | `CMD_ACK_FAULT` while `io_ok` is true: disarmed `MACHINE_IDLE`, fault cleared |

`start_allowed` is true only while the UI screen is the main screen. Opening the menu, settings, manual confirm, manual, stop confirm, or diagnostics sets it false, so a batch cannot start behind an open screen.

Zero timer values still consume one step. The state is entered on one call and the timer is evaluated on a later call.

### Command priority

On every step the core handles conditions in this order and then returns when a forced transition happens:

1. `io_ok == false` and the state is not `MACHINE_FAULT`: enter `MACHINE_FAULT`.
2. State is `MACHINE_FAULT`: `CMD_ACK_FAULT` with `io_ok == true` returns to disarmed idle. Any other command is ignored.
3. `CMD_STOP_CONFIRMED` while the state is not `MACHINE_MANUAL`: disarmed idle, outputs off, scale warning cleared.
4. `CMD_ENTER_MANUAL` while the state is `MACHINE_IDLE`: enter `MACHINE_MANUAL`.
5. State is `MACHINE_MANUAL`: apply `CMD_SET_SHREDDER`, `CMD_SET_GRINDER`, and `CMD_SET_DOOR`. `CMD_EXIT_MANUAL` returns to disarmed idle with outputs off. Automatic transitions do not run.
6. Update the stability window, then evaluate the current automatic state.

`CMD_ENTER_MANUAL` outside `MACHINE_IDLE` is ignored. `CMD_SET_*` outside `MACHINE_MANUAL` is ignored. `CMD_STOP_CONFIRMED` during `MACHINE_MANUAL` is ignored; leaving manual is `CMD_EXIT_MANUAL`. `CMD_ACK_FAULT` outside `MACHINE_FAULT`, or while the I/O link is still down, is ignored.

While the state is `MACHINE_WEIGHT_VALIDATION`, `MACHINE_START_GRINDER`, `MACHINE_START_SHREDDER`, `MACHINE_OPEN_DOOR`, `MACHINE_DISCHARGING`, `MACHINE_MOTOR_RUNON`, `MACHINE_CLOSE_DOOR`, or `MACHINE_BATCH_COMPLETE`, an invalid scale sample sets `scale_warning`. The flag stays set until re-arm, stop, fault acknowledgement, or entry to manual. The timing sequence still advances after the latch.

## Operator interface

Encoder levels are sampled from a GPIO interrupt and also polled every 10 ms so a held button reaches 800 ms. The decoder emits one event per call. Button events and completed detents that happen on the same sample are both delivered, one per call.

| Gesture | Result |
|---|---|
| Rotate one detent | Move a selection or change the selected value by one step |
| Short press | Activate the selection |
| Long press on the main screen in `MACHINE_IDLE` | Open the mode menu |
| Long press on the main screen in any other non-fault, non-manual state | Open stop confirmation |
| Long press on any other screen, in manual, or in fault | Ignored |
| Short press while the machine state is `MACHINE_FAULT` | Send `CMD_ACK_FAULT` |

Mode menu rows: Settings, Manual, Diagnostics.

Stop confirmation rows: Stop, Cancel. Stop sends `CMD_STOP_CONFIRMED` and returns to the main screen. Cancel returns to the main screen.

Manual confirmation rows: Enable, Cancel. Enable sends `CMD_ENTER_MANUAL` and opens the manual screen. The manual rows are Shredder, Grinder, Door, Back. A short press toggles an output and sends the matching `CMD_SET_*` with the new level. Back sends `CMD_EXIT_MANUAL` and returns to the main screen.

Settings rows are the twelve fields, then Save, then Back. A short press on a field arms edit; rotate changes it; a short press stores it in the screen copy. Save asks the HMI task to write NVS and, on success, publishes the new settings and returns to the menu. Back restores the settings captured when the screen opened and returns to the menu. The core receives new settings only after a successful save.

Diagnostics shows the scale snapshot, the I/O snapshot, the machine state, the active fault, and the active profile values (slave IDs, baud, scale register, decode name). A short press returns to the menu.

The main screen shows current weight, target weight, stable or unstable, shredder on or off, grinder on or off, door command open or close, the state name, scale link text, I/O link text, and a banner. Banner text is `Scale lost during batch` when `scale_warning` is set, `Settings reset to defaults` when `settings_reset` is set, and `Settings were not saved` when `settings_save_failed` is set. The fault screen replaces the other screens while the state is `MACHINE_FAULT` and shows `I/O link lost`.

State names returned by `machine_state_name`:

| State | Text |
|---|---|
| `MACHINE_IDLE` | Idle |
| `MACHINE_WEIGHT_VALIDATION` | Weight check |
| `MACHINE_START_GRINDER` | Start grinder |
| `MACHINE_START_SHREDDER` | Start shredder |
| `MACHINE_OPEN_DOOR` | Open door |
| `MACHINE_DISCHARGING` | Discharging |
| `MACHINE_CLOSE_DOOR` | Close door |
| `MACHINE_MOTOR_RUNON` | Motor run-on |
| `MACHINE_BATCH_COMPLETE` | Batch complete |
| `MACHINE_MANUAL` | Manual |
| `MACHINE_FAULT` | Fault |

Weight text is kilograms with two decimals. Time text is seconds with one decimal. The door row is labeled `Door command`.

Scale link text:

| Condition | Text |
|---|---|
| No successful sample yet | Scale -- |
| Latest grams negative | Scale invalid |
| Latest transaction failed | Scale lost |
| Sample older than 500 ms | Scale stale |
| Otherwise | Scale OK |

I/O link text is `I/O --` before the first attempt, `I/O lost` when an attempt has failed, and `I/O OK` after success.

Panel layout is 320 by 240. The initial profile swaps X and Y on the ILI9341 so that logical size is landscape. Mirror flags start false.

## Device profile

The initial compiled profile is the one the firmware runs until a commissioner edits `main/device/profile.c` and rebuilds. Diagnostics shows the active values.

| Field | Initial value |
|---|---|
| UART | UART 1, TX GPIO 17, RX GPIO 18, DE GPIO 16, DE used, 9600 baud, 8 data bits, no parity, 1 stop bit |
| UART mode | `UART_MODE_RS485_HALF_DUPLEX` with DE on the UART RTS pin |
| Scale | Slave 1, function `0x03`, register 0, count 2, decode `SCALE_DECODE_S32_BE_GRAMS` |
| I/O | Slave 2, function `0x0F`, coil 0, count 3. Bit 0 is the shredder, bit 1 the grinder, bit 2 the door |
| Panel | ILI9341, SCLK 4, MOSI 5, MISO 6, CS 7, DC 8, RST 9, backlight 10 active high, SPI2, 20 MHz, mode 0, RGB element order BGR, `swap_xy` true, mirrors false, 320 by 240 |
| Encoder | A GPIO 11, B GPIO 12, switch GPIO 13, switch active low, internal pull-ups |

`SCALE_DECODE_S32_BE_GRAMS` joins register 0 as the high 16 bits and register 1 as the low 16 bits into a signed 32-bit gram value. `SCALE_DECODE_S32_LE_GRAMS` swaps the words. `SCALE_DECODE_S16_GRAMS` uses register 0 as a signed 16-bit gram value and requires count 1. The initial profile uses the 32-bit big-endian decode.

Supported write functions are `0x0F` (one multiple-coil request) and `0x05` (one request per coil). The initial profile uses `0x0F`.

The master accepts function `0x03` and `0x04` for the scale. Status codes are `MODBUS_OK`, `MODBUS_ERR_TIMEOUT` (no bytes before the deadline), `MODBUS_ERR_CRC`, `MODBUS_ERR_SLAVE`, `MODBUS_ERR_FUNCTION`, `MODBUS_ERR_EXCEPTION` (function code has bit 7 set), `MODBUS_ERR_TRANSPORT` (write failed or read returned negative), `MODBUS_ERR_LENGTH` (frame shorter than 5 bytes or byte count does not match), and `MODBUS_ERR_ARGUMENT` (missing link callbacks, count 0, or count greater than 8). A failed scale transaction does not update the gram value.

## Modbus master

The master is given a transport:

```c
typedef struct {
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t *data, size_t max_len, uint32_t timeout_ms);
    void *ctx;
    uint32_t timeout_ms;
} modbus_link_t;
```

`write` returns 0 on success. `read` returns the number of bytes copied, 0 on timeout, and a negative value on a transport error. The master reads into a 64-byte buffer and parses the actual frame length. The board transport waits for UART TX done and flushes the RX buffer before returning from `write`, so the master reads only the reply. The board reader waits up to the response timeout for the first byte, then keeps reading until a 5 ms gap or the buffer is full. CRC-16/MODBUS is appended low byte first. Known vector: the bytes `123456789` produce `0x4B37`.

## Status bus

The bus holds mutex-protected copies of the machine status, scale snapshot, I/O snapshot, settings, `settings_reset`, `settings_save_failed`, and `start_allowed`. It also owns a FreeRTOS queue of `operator_event_t` with depth 8. `status_bus_take_command` is non-blocking and returns false when the queue is empty. Initial values: idle status, outputs off, `io.link_ok` true, `io.attempted` false, scale `last_ok` false, `have_sample` false, settings at the defaults, both settings flags false, `start_allowed` true.

## Boot order

1. `nvs_flash_init`.
2. Load settings. On failure, keep defaults and set `settings_reset`.
3. `status_bus_init` and publish settings.
4. Initialize SPI panel, backlight on, LVGL, and the encoder GPIO ISR.
5. Install the UART and set RS485 half duplex.
6. Create the HMI task, then the Modbus task, then the control task.

No Wi-Fi and no Bluetooth are started.

## Logging

Tags are `control`, `modbus`, and `hmi`. Control logs each state change at info level with `machine_state_name`. Modbus logs each failed transaction at warning level with the status code. HMI logs a failed NVS save at error level. The core and the UI logic do not log.

## File layout

```text
CMakeLists.txt
sdkconfig.defaults
partitions.csv
main/CMakeLists.txt
main/idf_component.yml
main/main.c
main/common/crc16.h
main/common/crc16.c
main/device/profile.h
main/device/profile.c
main/device/scale_decode.h
main/device/scale_decode.c
main/device/board.h
main/device/board.c
main/machine/machine.h
main/machine/machine_settings.c
main/machine/machine_core.c
main/machine/control_task.c
main/modbus/modbus_master.h
main/modbus/modbus_master.c
main/modbus/modbus_task.c
main/settings/settings_codec.h
main/settings/settings_codec.c
main/settings/settings_store.h
main/settings/settings_store.c
main/status/status_bus.h
main/status/status_bus.c
main/ui/encoder_decode.h
main/ui/encoder_decode.c
main/ui/ui_logic.h
main/ui/ui_logic.c
main/hmi/encoder_hw.h
main/hmi/encoder_hw.c
main/hmi/display.h
main/hmi/display.c
main/hmi/hmi_task.c
test/host/CMakeLists.txt
test/host/test_harness.h
test/host/test_crc16.c
test/host/test_settings.c
test/host/test_machine_core.c
test/host/test_modbus_master.c
test/host/test_scale_decode.c
test/host/test_encoder_decode.c
test/host/test_ui_logic.c
```

## Testing

Host tests cover CRC, the settings record, scale decode, the profile constants, the stability window, one full automatic cycle, independent run-on times, one-step-per-state, scale loss before and after the latch, re-arm, a single batch while weight stays high, I/O fault and acknowledgement, software stop, manual entry and exit, the latched cycle timers, `start_allowed`, encoder detents, long press versus short press, and the UI navigation rules.

`idf.py build` is the check that the ESP-IDF tasks, NVS store, UART transport, and LVGL screen compile. Commissioning on the machine (live scale, contactors, actuator, fish) follows the hardware stages already written in the project brief and is outside the host test list.

## Out of scope

- Motor protection sizing, contactor selection, breaker selection, and the electrical schematic.
- Hardwired emergency stop, overload contacts, and guarding.
- Editing baud, parity, slave IDs, or register addresses from the TFT.
- Door position feedback.
- Pilot lights, extra buttons, and extra sensors.
- Resuming a batch after power loss.
