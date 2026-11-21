# Fish Shredder and Grinder Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build flashable ESP-IDF 6.1.1 firmware that runs one weight-triggered fish batch, with the sequence proven by host tests before the board is wired.

**Architecture:** Plain C `machine_core` owns stability, arming, the state machine, and the desired outputs. A small Modbus RTU master is the only UART owner. Three FreeRTOS tasks (control, Modbus, HMI) exchange snapshots through a status bus. Weights and timers are integer grams and milliseconds in a versioned NVS record. The scale map and pins live in `main/device/profile.c`.

**Tech Stack:** C11, CMake, ESP-IDF v6.1.1, FreeRTOS, LVGL 9.5.0, `espressif/esp_lcd_ili9341` 2.1.0, NVS, ESP-IDF UART in RS485 half-duplex.

## Global Constraints

- ESP-IDF v6.1.1, target `esp32s3`, language C, CMake, FreeRTOS at 1000 Hz.
- No Arduino framework and no Arduino libraries.
- ESP32-S3 N16R8. Root `CMakeLists.txt` uses `include($ENV{IDF_PATH}/tools/cmake/project.cmake)` then `project(...)` so the build stays on build system v1. ESP-IDF v6.1.1 build system v2 fails the bootloader when `CONFIG_SPIRAM=y` (IDFGH-18754).
- LVGL 9.5.0 and `espressif/esp_lcd_ili9341` 2.1.0. The HMI task is the only caller of `lv_timer_handler`. Do not add `esp_lvgl_port`.
- Machine-core weights are integer grams. Time is a caller-supplied `uint32_t` millisecond count. Elapsed time is unsigned subtraction.
- At most one state transition per `machine_core_step` call. Zero durations still consume one step.
- Door command off means close. Boot, stop, fault, and leaving manual command shredder off, grinder off, and door close.
- Boot starts in `MACHINE_IDLE`, disarmed. An in-progress batch is never resumed.
- One operator command per step.
- Host tests compile with the system C compiler and include no ESP-IDF headers.
- Chip options live in `sdkconfig.defaults`. Slave IDs, baud, parity, pins, and the scale map live in `main/device/profile.c`.
- Design contract: `docs/superpowers/specs/2026-11-21-fish-shredder-grinder-firmware-design.md`.

---

## File map

Host-tested plain C lives under `main/` and is also compiled by `test/host/CMakeLists.txt`. ESP-IDF-only files (`control_task.c`, `modbus_task.c`, `settings_store.c`, `status_bus.c`, `board.c`, `encoder_hw.c`, `display.c`, `hmi_task.c`, `main.c`) stay out of the host library.

| File | Responsibility |
|---|---|
| `main/common/crc16.c` | CRC-16/MODBUS |
| `main/machine/machine_settings.c` | Defaults and `machine_settings_valid` |
| `main/machine/machine_core.c` | Sequence, stability, fault, manual |
| `main/settings/settings_codec.c` | 54-byte versioned record |
| `main/modbus/modbus_master.c` | RTU request, CRC check, status code |
| `main/device/scale_decode.c` | Register words to grams |
| `main/device/profile.c` | Initial compiled profile |
| `main/ui/encoder_decode.c` | Detents, short press, long press |
| `main/ui/ui_logic.c` | Screen navigation and commands |
| `main/machine/control_task.c` | 10 ms core step |
| `main/modbus/modbus_task.c` | Scale poll and output writes |
| `main/device/board.c` | UART RS485 transport and panel pins |
| `main/settings/settings_store.c` | NVS load and save |
| `main/status/status_bus.c` | Snapshots and command queue |
| `main/hmi/display.c` | ILI9341 plus LVGL flush |
| `main/hmi/encoder_hw.c` | GPIO ISR samples |
| `main/hmi/hmi_task.c` | Render, encoder, NVS save |
| `main/main.c` | Boot order |

Host test command, from the repo root, after each host task:

```powershell
cmake -S test/host -B build/host
cmake --build build/host --config Debug
ctest --test-dir build/host -C Debug --output-on-failure -R <test-name>
```

A passing run ends with `100% tests passed`. A task whose source file is not created yet fails the build because that `.c` file is missing.

### Task 1: CRC-16/MODBUS

**Files:**
- Create: `test/host/test_harness.h`
- Create: `test/host/CMakeLists.txt`
- Create: `test/host/test_crc16.c`
- Create: `main/common/crc16.h`
- Create: `main/common/crc16.c`
- Test: `test/host/test_crc16.c`

**Interfaces:**
- Consumes: nothing
- Produces: `uint16_t crc16_modbus(const uint8_t *data, size_t len);` in `main/common/crc16.h`. The bytes `123456789` return `0x4B37`. The frame bytes `01 03 00 00 00 0A` return `0xCDC5`.

- [ ] **Step 1: Write the failing test**

Create `test/host/test_harness.h`:

```c
#pragma once
#include <stdio.h>

extern int g_failures;

#define EXPECT(cond)                                                         \
    do {                                                                     \
        if (!(cond)) {                                                       \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)

#define EXPECT_EQ_INT(a, b)                                                  \
    do {                                                                     \
        long long _a = (long long)(a);                                       \
        long long _b = (long long)(b);                                       \
        if (_a != _b) {                                                      \
            fprintf(stderr, "FAIL %s:%d: %s (%lld) != %s (%lld)\n",          \
                    __FILE__, __LINE__, #a, _a, #b, _b);                     \
            g_failures++;                                                    \
        }                                                                    \
    } while (0)
```

Create `test/host/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(fish_shredder_host C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_library(firmware_host STATIC
    ${ROOT}/main/common/crc16.c
)
target_include_directories(firmware_host PUBLIC ${ROOT}/main)

add_executable(test_crc16 test_crc16.c)
target_link_libraries(test_crc16 PRIVATE firmware_host)
add_test(NAME test_crc16 COMMAND test_crc16)
```

Create `test/host/test_crc16.c`:

```c
#include "test_harness.h"
#include "common/crc16.h"

int g_failures = 0;

static void test_ascii_vector(void) {
    const uint8_t data[] = "123456789";
    EXPECT_EQ_INT(crc16_modbus(data, 9), 0x4B37);
}

static void test_read_request_vector(void) {
    const uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    EXPECT_EQ_INT(crc16_modbus(data, sizeof data), 0xCDC5);
}

static void test_empty_is_initial_value(void) {
    EXPECT_EQ_INT(crc16_modbus(NULL, 0), 0xFFFF);
}

int main(void) {
    test_ascii_vector();
    test_read_request_vector();
    test_empty_is_initial_value();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: configuration or generation fails because `main/common/crc16.c` does not exist. The error text contains `crc16.c`.

- [ ] **Step 3: Write the minimal implementation**

Create `main/common/crc16.h`:

```c
#pragma once
#include <stddef.h>
#include <stdint.h>

uint16_t crc16_modbus(const uint8_t *data, size_t len);
```

Create `main/common/crc16.c`:

```c
#include "common/crc16.h"

uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if ((crc & 1u) != 0u) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure -R test_crc16`

Expected: `100% tests passed` and the program prints `ok`.

- [ ] **Step 5: Commit**

```bash
git add main/common/crc16.h main/common/crc16.c test/host/test_harness.h test/host/test_crc16.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add the Modbus CRC-16 used by frames and settings records.

EOF
)"
```

### Task 2: Settings validation and codec

**Files:**
- Create: `main/machine/machine.h`
- Create: `main/machine/machine_settings.c`
- Create: `main/settings/settings_codec.h`
- Create: `main/settings/settings_codec.c`
- Create: `test/host/test_settings.c`
- Modify: `test/host/CMakeLists.txt`
- Test: `test/host/test_settings.c`

**Interfaces:**
- Consumes: `crc16_modbus`
- Produces:
  - `machine_settings_t` and the enums and structs in `main/machine/machine.h` listed in Step 3
  - `machine_settings_t machine_settings_defaults(void);`
  - `bool machine_settings_valid(const machine_settings_t *settings);`
  - `size_t settings_codec_size(void);` returns 54
  - `bool settings_codec_pack(const machine_settings_t *in, uint8_t *out, size_t out_len);`
  - `bool settings_codec_unpack(const uint8_t *in, size_t len, machine_settings_t *out);`

- [ ] **Step 1: Write the failing test**

Create `test/host/test_settings.c`:

```c
#include "test_harness.h"
#include "machine/machine.h"
#include "settings/settings_codec.h"

#include <string.h>

int g_failures = 0;

static void test_defaults_are_valid(void) {
    machine_settings_t s = machine_settings_defaults();
    EXPECT(machine_settings_valid(&s));
    EXPECT_EQ_INT(s.target_g, 25000);
    EXPECT_EQ_INT(s.minimum_batch_g, 1000);
    EXPECT_EQ_INT(s.tolerance_g, 100);
    EXPECT_EQ_INT(s.stable_ms, 2000);
    EXPECT_EQ_INT(s.reset_threshold_g, 500);
    EXPECT_EQ_INT(s.grinder_startup_ms, 2000);
    EXPECT_EQ_INT(s.shredder_startup_ms, 2000);
    EXPECT_EQ_INT(s.door_open_ms, 5000);
    EXPECT_EQ_INT(s.discharge_ms, 10000);
    EXPECT_EQ_INT(s.door_close_ms, 5000);
    EXPECT_EQ_INT(s.shredder_run_on_ms, 5000);
    EXPECT_EQ_INT(s.grinder_run_on_ms, 8000);
}

static void test_rejects_reset_not_below_target(void) {
    machine_settings_t s = machine_settings_defaults();
    s.reset_threshold_g = s.target_g;
    EXPECT(!machine_settings_valid(&s));
}

static void test_rejects_minimum_above_target(void) {
    machine_settings_t s = machine_settings_defaults();
    s.minimum_batch_g = s.target_g + 1;
    EXPECT(!machine_settings_valid(&s));
}

static void test_rejects_null_and_zero_tolerance(void) {
    EXPECT(!machine_settings_valid(NULL));
    machine_settings_t s = machine_settings_defaults();
    s.tolerance_g = 0;
    EXPECT(!machine_settings_valid(&s));
    s = machine_settings_defaults();
    s.stable_ms = 0;
    EXPECT(!machine_settings_valid(&s));
}

static void test_codec_round_trip(void) {
    machine_settings_t in = machine_settings_defaults();
    in.target_g = 18000;
    uint8_t buf[54];
    EXPECT_EQ_INT(settings_codec_size(), 54);
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    machine_settings_t out;
    memset(&out, 0, sizeof out);
    EXPECT(settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT_EQ_INT(out.target_g, 18000);
    EXPECT_EQ_INT(out.grinder_run_on_ms, 8000);
}

static void test_codec_rejects_bad_crc_and_version(void) {
    machine_settings_t in = machine_settings_defaults();
    uint8_t buf[54];
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    buf[10] ^= 0xFFu;
    machine_settings_t out = machine_settings_defaults();
    EXPECT(!settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    buf[0] = 99;
    EXPECT(!settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT(!settings_codec_unpack(buf, 10, &out));
    EXPECT(!settings_codec_pack(NULL, buf, sizeof buf));
}

int main(void) {
    test_defaults_are_valid();
    test_rejects_reset_not_below_target();
    test_rejects_minimum_above_target();
    test_rejects_null_and_zero_tolerance();
    test_codec_round_trip();
    test_codec_rejects_bad_crc_and_version();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
```

Replace `test/host/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.16)
project(fish_shredder_host C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_library(firmware_host STATIC
    ${ROOT}/main/common/crc16.c
    ${ROOT}/main/machine/machine_settings.c
    ${ROOT}/main/settings/settings_codec.c
)
target_include_directories(firmware_host PUBLIC ${ROOT}/main)

function(add_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE firmware_host)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_crc16)
add_host_test(test_settings)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: failure text contains `machine_settings.c` or `settings_codec.c`.

- [ ] **Step 3: Write the minimal implementation**

Create `main/machine/machine.h`:

```c
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MACHINE_IDLE = 0,
    MACHINE_WEIGHT_VALIDATION,
    MACHINE_START_GRINDER,
    MACHINE_START_SHREDDER,
    MACHINE_OPEN_DOOR,
    MACHINE_DISCHARGING,
    MACHINE_CLOSE_DOOR,
    MACHINE_MOTOR_RUNON,
    MACHINE_BATCH_COMPLETE,
    MACHINE_MANUAL,
    MACHINE_FAULT
} machine_state_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_IO_LINK
} fault_cause_t;

typedef enum {
    CMD_NONE = 0,
    CMD_ACK_FAULT,
    CMD_STOP_CONFIRMED,
    CMD_ENTER_MANUAL,
    CMD_EXIT_MANUAL,
    CMD_SET_SHREDDER,
    CMD_SET_GRINDER,
    CMD_SET_DOOR
} operator_cmd_t;

typedef struct {
    operator_cmd_t type;
    bool level;
} operator_event_t;

typedef struct {
    bool shredder;
    bool grinder;
    bool door_open;
} output_image_t;

typedef struct {
    int32_t grams;
    bool valid;
} scale_sample_t;

typedef struct {
    int32_t target_g;
    int32_t minimum_batch_g;
    int32_t tolerance_g;
    uint32_t stable_ms;
    int32_t reset_threshold_g;
    uint32_t grinder_startup_ms;
    uint32_t shredder_startup_ms;
    uint32_t door_open_ms;
    uint32_t discharge_ms;
    uint32_t door_close_ms;
    uint32_t shredder_run_on_ms;
    uint32_t grinder_run_on_ms;
} machine_settings_t;

typedef struct {
    machine_state_t state;
    fault_cause_t fault;
    output_image_t outputs;
    bool batch_armed;
    bool scale_warning;
    bool stable;
    uint32_t stable_elapsed_ms;
    int32_t latest_g;
    int32_t trigger_weight_g;
} machine_status_t;

typedef struct {
    machine_settings_t settings;
    machine_settings_t cycle;
    machine_status_t status;
    uint32_t state_entered_ms;
    int32_t window_min_g;
    int32_t window_max_g;
    uint32_t window_start_ms;
    bool window_open;
} machine_core_t;

machine_settings_t machine_settings_defaults(void);
bool machine_settings_valid(const machine_settings_t *settings);

bool machine_core_init(machine_core_t *core, const machine_settings_t *settings);
bool machine_core_set_settings(machine_core_t *core, const machine_settings_t *settings);
void machine_core_step(machine_core_t *core, uint32_t now_ms, scale_sample_t scale,
                       bool io_ok, bool start_allowed, const operator_event_t *command);
const machine_status_t *machine_core_status(const machine_core_t *core);
const char *machine_state_name(machine_state_t state);
```

Create `main/machine/machine_settings.c`:

```c
#include "machine/machine.h"

machine_settings_t machine_settings_defaults(void) {
    machine_settings_t s = {
        .target_g = 25000,
        .minimum_batch_g = 1000,
        .tolerance_g = 100,
        .stable_ms = 2000,
        .reset_threshold_g = 500,
        .grinder_startup_ms = 2000,
        .shredder_startup_ms = 2000,
        .door_open_ms = 5000,
        .discharge_ms = 10000,
        .door_close_ms = 5000,
        .shredder_run_on_ms = 5000,
        .grinder_run_on_ms = 8000,
    };
    return s;
}

bool machine_settings_valid(const machine_settings_t *settings) {
    if (settings == NULL) {
        return false;
    }
    if (settings->target_g <= 0 || settings->minimum_batch_g <= 0) {
        return false;
    }
    if (settings->minimum_batch_g > settings->target_g) {
        return false;
    }
    if (settings->tolerance_g <= 0 || settings->stable_ms == 0) {
        return false;
    }
    if (settings->reset_threshold_g < 0 || settings->reset_threshold_g >= settings->target_g) {
        return false;
    }
    return true;
}
```

Create `main/settings/settings_codec.h`:

```c
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "machine/machine.h"

size_t settings_codec_size(void);
bool settings_codec_pack(const machine_settings_t *in, uint8_t *out, size_t out_len);
bool settings_codec_unpack(const uint8_t *in, size_t len, machine_settings_t *out);
```

Create `main/settings/settings_codec.c`:

```c
#include "settings/settings_codec.h"
#include "common/crc16.h"

#include <string.h>

#define SETTINGS_VERSION 1u
#define SETTINGS_BYTES 54u

size_t settings_codec_size(void) {
    return SETTINGS_BYTES;
}

static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_fields(uint8_t *p, const machine_settings_t *in) {
    put_u32(p + 0, (uint32_t)in->target_g);
    put_u32(p + 4, (uint32_t)in->minimum_batch_g);
    put_u32(p + 8, (uint32_t)in->tolerance_g);
    put_u32(p + 12, in->stable_ms);
    put_u32(p + 16, (uint32_t)in->reset_threshold_g);
    put_u32(p + 20, in->grinder_startup_ms);
    put_u32(p + 24, in->shredder_startup_ms);
    put_u32(p + 28, in->door_open_ms);
    put_u32(p + 32, in->discharge_ms);
    put_u32(p + 36, in->door_close_ms);
    put_u32(p + 40, in->shredder_run_on_ms);
    put_u32(p + 44, in->grinder_run_on_ms);
}

static void read_fields(const uint8_t *p, machine_settings_t *out) {
    out->target_g = (int32_t)get_u32(p + 0);
    out->minimum_batch_g = (int32_t)get_u32(p + 4);
    out->tolerance_g = (int32_t)get_u32(p + 8);
    out->stable_ms = get_u32(p + 12);
    out->reset_threshold_g = (int32_t)get_u32(p + 16);
    out->grinder_startup_ms = get_u32(p + 20);
    out->shredder_startup_ms = get_u32(p + 24);
    out->door_open_ms = get_u32(p + 28);
    out->discharge_ms = get_u32(p + 32);
    out->door_close_ms = get_u32(p + 36);
    out->shredder_run_on_ms = get_u32(p + 40);
    out->grinder_run_on_ms = get_u32(p + 44);
}

bool settings_codec_pack(const machine_settings_t *in, uint8_t *out, size_t out_len) {
    if (in == NULL || out == NULL || out_len < SETTINGS_BYTES || !machine_settings_valid(in)) {
        return false;
    }
    memset(out, 0, SETTINGS_BYTES);
    put_u32(out, SETTINGS_VERSION);
    write_fields(out + 4, in);
    uint16_t crc = crc16_modbus(out, 52);
    out[52] = (uint8_t)crc;
    out[53] = (uint8_t)(crc >> 8);
    return true;
}

bool settings_codec_unpack(const uint8_t *in, size_t len, machine_settings_t *out) {
    if (in == NULL || out == NULL || len != SETTINGS_BYTES) {
        return false;
    }
    if (get_u32(in) != SETTINGS_VERSION) {
        return false;
    }
    uint16_t got = (uint16_t)(in[52] | (in[53] << 8));
    if (got != crc16_modbus(in, 52)) {
        return false;
    }
    machine_settings_t decoded;
    memset(&decoded, 0, sizeof decoded);
    read_fields(in + 4, &decoded);
    if (!machine_settings_valid(&decoded)) {
        return false;
    }
    *out = decoded;
    return true;
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure -R "test_crc16|test_settings"`

Expected: `100% tests passed`.

- [ ] **Step 5: Commit**

```bash
git add main/machine/machine.h main/machine/machine_settings.c main/settings/settings_codec.h main/settings/settings_codec.c test/host/test_settings.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add versioned settings records with the batch defaults.

EOF
)"
```

### Task 3: Machine core

**Files:**
- Create: `main/machine/machine_core.c`
- Create: `test/host/test_machine_core.c`
- Modify: `test/host/CMakeLists.txt`
- Test: `test/host/test_machine_core.c`

**Interfaces:**
- Consumes: `machine_settings_defaults`, `machine_settings_valid`, and the types in `machine.h`
- Produces: `machine_core_init`, `machine_core_set_settings`, `machine_core_step`, `machine_core_status`, `machine_state_name` with the behavior in the design spec

- [ ] **Step 1: Write the failing test**

Add `${ROOT}/main/machine/machine_core.c` to the `firmware_host` sources and add `add_host_test(test_machine_core)` to `test/host/CMakeLists.txt`.

Create `test/host/test_machine_core.c`:

```c
#include "test_harness.h"
#include "machine/machine.h"

int g_failures = 0;

static const machine_status_t *st(const machine_core_t *core) {
    return machine_core_status(core);
}

static void feed(machine_core_t *core, uint32_t *now, int32_t grams, bool valid,
                 bool io_ok, bool start_allowed, const operator_event_t *cmd,
                 uint32_t duration_ms) {
    uint32_t start = *now;
    const operator_event_t *next = cmd;
    while ((*now - start) <= duration_ms) {
        scale_sample_t sample = {.grams = grams, .valid = valid};
        machine_core_step(core, *now, sample, io_ok, start_allowed, next);
        next = NULL;
        *now += 10;
    }
}

static void once(machine_core_t *core, uint32_t *now, int32_t grams, bool valid,
                 bool io_ok, bool start_allowed, const operator_event_t *cmd) {
    scale_sample_t sample = {.grams = grams, .valid = valid};
    machine_core_step(core, *now, sample, io_ok, start_allowed, cmd);
    *now += 10;
}

static uint32_t step_until(machine_core_t *core, uint32_t *now, int32_t grams,
                           machine_state_t target, int limit) {
    for (int i = 0; i < limit; i++) {
        uint32_t at = *now;
        once(core, now, grams, true, true, true, NULL);
        if (st(core)->state == target) {
            return at;
        }
    }
    EXPECT(0);
    return 0;
}

static void arm_at(machine_core_t *core, uint32_t *now, uint32_t stable_ms) {
    feed(core, now, 100, true, true, true, NULL, stable_ms);
    EXPECT(st(core)->batch_armed);
    EXPECT_EQ_INT(st(core)->state, MACHINE_IDLE);
}

static void test_boot_outputs_off_and_disarmed(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    EXPECT(machine_core_init(&core, &s));
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->batch_armed);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.door_open);
    EXPECT_EQ_INT(st(&core)->fault, FAULT_NONE);
    const char *name = machine_state_name(MACHINE_IDLE);
    EXPECT(name[0] == 'I' && name[1] == 'd' && name[2] == 'l' && name[3] == 'e' && name[4] == '\0');
}

static void test_invalid_settings_fall_back_to_defaults(void) {
    machine_core_t core;
    machine_settings_t bad = machine_settings_defaults();
    bad.tolerance_g = 0;
    EXPECT(!machine_core_init(&core, &bad));
    EXPECT_EQ_INT(core.settings.target_g, 25000);
    bad = machine_settings_defaults();
    EXPECT(machine_core_init(&core, &bad));
    bad.reset_threshold_g = bad.target_g;
    EXPECT(!machine_core_set_settings(&core, &bad));
    EXPECT_EQ_INT(core.settings.reset_threshold_g, 500);
}

static void test_stable_when_spread_within_tolerance(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    feed(&core, &now, 26000, true, true, false, NULL, 1900);
    EXPECT(!st(&core)->stable);
    feed(&core, &now, 26050, true, true, false, NULL, 100);
    EXPECT(st(&core)->stable);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->batch_armed);
}

static void test_window_resets_when_spread_exceeds_tolerance(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    feed(&core, &now, 26000, true, true, false, NULL, 0);
    feed(&core, &now, 26200, true, true, false, NULL, 1990);
    EXPECT(!st(&core)->stable);
    feed(&core, &now, 26200, true, true, false, NULL, 10);
    EXPECT(st(&core)->stable);
}

static void test_not_ready_when_latest_below_target(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 25050, true, true, true, NULL, 1000);
    feed(&core, &now, 24900, true, true, true, NULL, 2000);
    EXPECT(st(&core)->stable);
    EXPECT(st(&core)->batch_armed);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
}

static void test_overshoot_runs_one_cycle(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_WEIGHT_VALIDATION);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT_EQ_INT(st(&core)->trigger_weight_g, 26400);
    uint32_t grinder_at = step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    EXPECT(st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.door_open);
    uint32_t shredder_at = step_until(&core, &now, 26400, MACHINE_START_SHREDDER, 400);
    EXPECT_EQ_INT(shredder_at - grinder_at, 2000);
    EXPECT(st(&core)->outputs.shredder);
    step_until(&core, &now, 26400, MACHINE_OPEN_DOOR, 400);
    EXPECT(st(&core)->outputs.door_open);
    step_until(&core, &now, 26400, MACHINE_DISCHARGING, 800);
    EXPECT(st(&core)->outputs.door_open);
    step_until(&core, &now, 26400, MACHINE_CLOSE_DOOR, 1500);
    EXPECT(!st(&core)->outputs.door_open);
    EXPECT(st(&core)->outputs.grinder);
    EXPECT(st(&core)->outputs.shredder);
    (void)step_until(&core, &now, 26400, MACHINE_MOTOR_RUNON, 800);
    (void)step_until(&core, &now, 26400, MACHINE_BATCH_COMPLETE, 1200);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.door_open);
    EXPECT_EQ_INT(machine_state_name(MACHINE_BATCH_COMPLETE)[0], 'B');
}

static void test_shredder_stops_before_grinder(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    (void)step_until(&core, &now, 26400, MACHINE_MOTOR_RUNON, 4000);
    uint32_t entered = now - 10;
    while (st(&core)->outputs.shredder) {
        once(&core, &now, 26400, true, true, true, NULL);
    }
    EXPECT_EQ_INT((now - 10) - entered, 5000);
    EXPECT(st(&core)->outputs.grinder);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_MOTOR_RUNON);
    while (st(&core)->state != MACHINE_BATCH_COMPLETE) {
        once(&core, &now, 26400, true, true, true, NULL);
    }
    EXPECT_EQ_INT((now - 10) - entered, 8000);
    EXPECT(!st(&core)->outputs.grinder);
}

static void test_zero_delay_one_step_per_state(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    s.stable_ms = 500;
    s.grinder_startup_ms = 0;
    s.shredder_startup_ms = 0;
    s.door_open_ms = 0;
    s.discharge_ms = 0;
    s.door_close_ms = 0;
    s.shredder_run_on_ms = 0;
    s.grinder_run_on_ms = 0;
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 500);
    feed(&core, &now, 26000, true, true, true, NULL, 500);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_WEIGHT_VALIDATION);
    machine_state_t seen[8];
    for (int i = 0; i < 8; i++) {
        once(&core, &now, 26000, true, true, true, NULL);
        seen[i] = st(&core)->state;
    }
    EXPECT_EQ_INT(seen[0], MACHINE_START_GRINDER);
    EXPECT_EQ_INT(seen[1], MACHINE_START_SHREDDER);
    EXPECT_EQ_INT(seen[2], MACHINE_OPEN_DOOR);
    EXPECT_EQ_INT(seen[3], MACHINE_DISCHARGING);
    EXPECT_EQ_INT(seen[4], MACHINE_CLOSE_DOOR);
    EXPECT_EQ_INT(seen[5], MACHINE_MOTOR_RUNON);
    EXPECT_EQ_INT(seen[6], MACHINE_BATCH_COMPLETE);
    EXPECT_EQ_INT(seen[7], MACHINE_BATCH_COMPLETE);
}

static void test_scale_loss_before_latch_does_not_start(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    feed(&core, &now, 30000, false, true, true, NULL, 5000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->batch_armed);
    arm_at(&core, &now, 2000);
    feed(&core, &now, 30000, false, true, true, NULL, 5000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(st(&core)->batch_armed);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_WEIGHT_VALIDATION);
}

static void test_scale_loss_after_latch_finishes_and_blocks_rearm(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    feed(&core, &now, 0, false, true, true, NULL, 60000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_BATCH_COMPLETE);
    EXPECT(st(&core)->scale_warning);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.door_open);
    feed(&core, &now, 0, false, true, true, NULL, 3000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_BATCH_COMPLETE);
    feed(&core, &now, 0, true, true, true, NULL, 2000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(st(&core)->batch_armed);
    EXPECT(!st(&core)->scale_warning);
}

static void test_weight_stays_high_does_not_start_second_batch(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_BATCH_COMPLETE, 8000);
    feed(&core, &now, 26400, true, true, true, NULL, 5000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_BATCH_COMPLETE);
    EXPECT(!st(&core)->batch_armed);
}

static void test_io_fault_and_ack(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    once(&core, &now, 26400, true, false, true, NULL);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_FAULT);
    EXPECT_EQ_INT(st(&core)->fault, FAULT_IO_LINK);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.door_open);
    operator_event_t ack = {.type = CMD_ACK_FAULT, .level = false};
    once(&core, &now, 26400, true, false, true, &ack);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_FAULT);
    once(&core, &now, 26400, true, true, true, &ack);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->batch_armed);
    EXPECT_EQ_INT(st(&core)->fault, FAULT_NONE);
}

static void test_stop_cancels_cycle(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_DISCHARGING, 4000);
    EXPECT(st(&core)->outputs.door_open);
    operator_event_t stop = {.type = CMD_STOP_CONFIRMED, .level = false};
    once(&core, &now, 26400, true, true, true, &stop);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->outputs.door_open);
    EXPECT(!st(&core)->outputs.grinder);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->batch_armed);
    feed(&core, &now, 26400, true, true, true, NULL, 5000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
}

static void test_manual_only_from_idle_and_exit_forces_off(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    operator_event_t enter = {.type = CMD_ENTER_MANUAL, .level = false};
    once(&core, &now, 0, true, true, true, &enter);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_MANUAL);
    EXPECT(!st(&core)->batch_armed);
    operator_event_t shredder = {.type = CMD_SET_SHREDDER, .level = true};
    operator_event_t door = {.type = CMD_SET_DOOR, .level = true};
    once(&core, &now, 0, true, true, true, &shredder);
    once(&core, &now, 0, true, true, true, &door);
    EXPECT(st(&core)->outputs.shredder);
    EXPECT(st(&core)->outputs.door_open);
    EXPECT(!st(&core)->outputs.grinder);
    operator_event_t leave = {.type = CMD_EXIT_MANUAL, .level = false};
    once(&core, &now, 0, true, true, true, &leave);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(!st(&core)->outputs.shredder);
    EXPECT(!st(&core)->outputs.door_open);
    EXPECT(!st(&core)->batch_armed);

    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    (void)step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    once(&core, &now, 26400, true, true, true, &enter);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_START_GRINDER);
    EXPECT(st(&core)->outputs.grinder);
}

static void test_settings_change_does_not_alter_latched_cycle(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, true, NULL, 2000);
    uint32_t grinder_at = step_until(&core, &now, 26400, MACHINE_START_GRINDER, 5);
    s.grinder_startup_ms = 9000;
    EXPECT(machine_core_set_settings(&core, &s));
    uint32_t shredder_at = step_until(&core, &now, 26400, MACHINE_START_SHREDDER, 400);
    EXPECT_EQ_INT(shredder_at - grinder_at, 2000);
}

static void test_start_not_allowed_blocks_start(void) {
    machine_core_t core;
    machine_settings_t s = machine_settings_defaults();
    machine_core_init(&core, &s);
    uint32_t now = 0;
    arm_at(&core, &now, 2000);
    feed(&core, &now, 26400, true, true, false, NULL, 2000);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_IDLE);
    EXPECT(st(&core)->batch_armed);
    once(&core, &now, 26400, true, true, true, NULL);
    EXPECT_EQ_INT(st(&core)->state, MACHINE_WEIGHT_VALIDATION);
}

static void test_state_names(void) {
    EXPECT_EQ_INT(machine_state_name(MACHINE_START_GRINDER)[0], 'S');
    EXPECT_EQ_INT(machine_state_name(MACHINE_FAULT)[0], 'F');
    EXPECT_EQ_INT(machine_state_name(MACHINE_MANUAL)[0], 'M');
}

int main(void) {
    test_boot_outputs_off_and_disarmed();
    test_invalid_settings_fall_back_to_defaults();
    test_stable_when_spread_within_tolerance();
    test_window_resets_when_spread_exceeds_tolerance();
    test_not_ready_when_latest_below_target();
    test_overshoot_runs_one_cycle();
    test_shredder_stops_before_grinder();
    test_zero_delay_one_step_per_state();
    test_scale_loss_before_latch_does_not_start();
    test_scale_loss_after_latch_finishes_and_blocks_rearm();
    test_weight_stays_high_does_not_start_second_batch();
    test_io_fault_and_ack();
    test_stop_cancels_cycle();
    test_manual_only_from_idle_and_exit_forces_off();
    test_settings_change_does_not_alter_latched_cycle();
    test_start_not_allowed_blocks_start();
    test_state_names();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: failure text contains `machine_core.c`.

- [ ] **Step 3: Write the minimal implementation**

Create `main/machine/machine_core.c`:

```c
#include "machine/machine.h"

#include <string.h>

static void outputs_off(machine_core_t *core) {
    core->status.outputs.shredder = false;
    core->status.outputs.grinder = false;
    core->status.outputs.door_open = false;
}

static void clear_window(machine_core_t *core) {
    core->window_open = false;
    core->window_min_g = 0;
    core->window_max_g = 0;
    core->window_start_ms = 0;
    core->status.stable = false;
    core->status.stable_elapsed_ms = 0;
}

static void enter_state(machine_core_t *core, machine_state_t state, uint32_t now_ms) {
    core->status.state = state;
    core->state_entered_ms = now_ms;
}

static void enter_idle_disarmed(machine_core_t *core, uint32_t now_ms) {
    outputs_off(core);
    core->status.batch_armed = false;
    core->status.scale_warning = false;
    core->status.fault = FAULT_NONE;
    core->status.trigger_weight_g = 0;
    clear_window(core);
    enter_state(core, MACHINE_IDLE, now_ms);
}

static void enter_fault(machine_core_t *core, uint32_t now_ms) {
    outputs_off(core);
    core->status.fault = FAULT_IO_LINK;
    enter_state(core, MACHINE_FAULT, now_ms);
}

static void enter_manual(machine_core_t *core, uint32_t now_ms) {
    outputs_off(core);
    core->status.batch_armed = false;
    core->status.scale_warning = false;
    core->status.fault = FAULT_NONE;
    clear_window(core);
    enter_state(core, MACHINE_MANUAL, now_ms);
}

static bool sample_ok(scale_sample_t scale) {
    return scale.valid && scale.grams >= 0;
}

static void update_stability(machine_core_t *core, uint32_t now_ms, scale_sample_t scale) {
    if (!sample_ok(scale)) {
        clear_window(core);
        return;
    }
    core->status.latest_g = scale.grams;
    if (!core->window_open) {
        core->window_open = true;
        core->window_min_g = scale.grams;
        core->window_max_g = scale.grams;
        core->window_start_ms = now_ms;
    } else {
        if (scale.grams < core->window_min_g) {
            core->window_min_g = scale.grams;
        }
        if (scale.grams > core->window_max_g) {
            core->window_max_g = scale.grams;
        }
        if ((core->window_max_g - core->window_min_g) > core->settings.tolerance_g) {
            core->window_min_g = scale.grams;
            core->window_max_g = scale.grams;
            core->window_start_ms = now_ms;
        }
    }
    core->status.stable_elapsed_ms = now_ms - core->window_start_ms;
    core->status.stable = core->status.stable_elapsed_ms >= core->settings.stable_ms;
}

static bool batch_ready(const machine_core_t *core, scale_sample_t scale) {
    return sample_ok(scale) && scale.grams >= core->settings.minimum_batch_g &&
           scale.grams >= core->settings.target_g && core->status.stable;
}

static bool rearm_ready(const machine_core_t *core, scale_sample_t scale) {
    return sample_ok(scale) && scale.grams < core->settings.reset_threshold_g && core->status.stable;
}

static bool warns_on_scale_loss(machine_state_t state) {
    return state == MACHINE_WEIGHT_VALIDATION || state == MACHINE_START_GRINDER ||
           state == MACHINE_START_SHREDDER || state == MACHINE_OPEN_DOOR ||
           state == MACHINE_DISCHARGING || state == MACHINE_CLOSE_DOOR ||
           state == MACHINE_MOTOR_RUNON || state == MACHINE_BATCH_COMPLETE;
}

static void begin_validation(machine_core_t *core, uint32_t now_ms) {
    core->cycle = core->settings;
    core->status.batch_armed = false;
    core->status.trigger_weight_g = core->status.latest_g;
    core->status.scale_warning = false;
    enter_state(core, MACHINE_WEIGHT_VALIDATION, now_ms);
}

static void begin_grinder(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = false;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = false;
    enter_state(core, MACHINE_START_GRINDER, now_ms);
}

static void begin_shredder(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = true;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = false;
    enter_state(core, MACHINE_START_SHREDDER, now_ms);
}

static void begin_open(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = true;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = true;
    enter_state(core, MACHINE_OPEN_DOOR, now_ms);
}

static void begin_discharge(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = true;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = true;
    enter_state(core, MACHINE_DISCHARGING, now_ms);
}

static void begin_close(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = true;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = false;
    enter_state(core, MACHINE_CLOSE_DOOR, now_ms);
}

static void begin_runon(machine_core_t *core, uint32_t now_ms) {
    core->status.outputs.shredder = true;
    core->status.outputs.grinder = true;
    core->status.outputs.door_open = false;
    enter_state(core, MACHINE_MOTOR_RUNON, now_ms);
}

static bool due(const machine_core_t *core, uint32_t now_ms, uint32_t duration_ms) {
    return (now_ms - core->state_entered_ms) >= duration_ms;
}

bool machine_core_init(machine_core_t *core, const machine_settings_t *settings) {
    memset(core, 0, sizeof(*core));
    bool accepted = settings != NULL && machine_settings_valid(settings);
    core->settings = accepted ? *settings : machine_settings_defaults();
    core->cycle = core->settings;
    core->status.state = MACHINE_IDLE;
    core->status.fault = FAULT_NONE;
    return accepted;
}

bool machine_core_set_settings(machine_core_t *core, const machine_settings_t *settings) {
    if (settings == NULL || !machine_settings_valid(settings)) {
        return false;
    }
    core->settings = *settings;
    return true;
}

const machine_status_t *machine_core_status(const machine_core_t *core) {
    return &core->status;
}

const char *machine_state_name(machine_state_t state) {
    switch (state) {
    case MACHINE_IDLE: return "Idle";
    case MACHINE_WEIGHT_VALIDATION: return "Weight check";
    case MACHINE_START_GRINDER: return "Start grinder";
    case MACHINE_START_SHREDDER: return "Start shredder";
    case MACHINE_OPEN_DOOR: return "Open door";
    case MACHINE_DISCHARGING: return "Discharging";
    case MACHINE_CLOSE_DOOR: return "Close door";
    case MACHINE_MOTOR_RUNON: return "Motor run-on";
    case MACHINE_BATCH_COMPLETE: return "Batch complete";
    case MACHINE_MANUAL: return "Manual";
    case MACHINE_FAULT: return "Fault";
    default: return "Unknown";
    }
}

void machine_core_step(machine_core_t *core, uint32_t now_ms, scale_sample_t scale,
                       bool io_ok, bool start_allowed, const operator_event_t *command) {
    operator_event_t none = {.type = CMD_NONE, .level = false};
    if (command == NULL) {
        command = &none;
    }

    if (!io_ok && core->status.state != MACHINE_FAULT) {
        enter_fault(core, now_ms);
        return;
    }

    if (core->status.state == MACHINE_FAULT) {
        if (command->type == CMD_ACK_FAULT && io_ok) {
            enter_idle_disarmed(core, now_ms);
        }
        return;
    }

    if (command->type == CMD_STOP_CONFIRMED && core->status.state != MACHINE_MANUAL) {
        enter_idle_disarmed(core, now_ms);
        return;
    }

    if (command->type == CMD_ENTER_MANUAL && core->status.state == MACHINE_IDLE) {
        enter_manual(core, now_ms);
        return;
    }

    if (core->status.state == MACHINE_MANUAL) {
        if (command->type == CMD_EXIT_MANUAL) {
            enter_idle_disarmed(core, now_ms);
            return;
        }
        if (command->type == CMD_SET_SHREDDER) {
            core->status.outputs.shredder = command->level;
        } else if (command->type == CMD_SET_GRINDER) {
            core->status.outputs.grinder = command->level;
        } else if (command->type == CMD_SET_DOOR) {
            core->status.outputs.door_open = command->level;
        }
        return;
    }

    update_stability(core, now_ms, scale);
    if (!sample_ok(scale) && warns_on_scale_loss(core->status.state)) {
        core->status.scale_warning = true;
    }

    switch (core->status.state) {
    case MACHINE_IDLE:
        if (!core->status.batch_armed && rearm_ready(core, scale)) {
            core->status.batch_armed = true;
            core->status.scale_warning = false;
            clear_window(core);
        } else if (core->status.batch_armed && start_allowed && batch_ready(core, scale)) {
            begin_validation(core, now_ms);
        }
        break;
    case MACHINE_WEIGHT_VALIDATION:
        begin_grinder(core, now_ms);
        break;
    case MACHINE_START_GRINDER:
        if (due(core, now_ms, core->cycle.grinder_startup_ms)) {
            begin_shredder(core, now_ms);
        }
        break;
    case MACHINE_START_SHREDDER:
        if (due(core, now_ms, core->cycle.shredder_startup_ms)) {
            begin_open(core, now_ms);
        }
        break;
    case MACHINE_OPEN_DOOR:
        if (due(core, now_ms, core->cycle.door_open_ms)) {
            begin_discharge(core, now_ms);
        }
        break;
    case MACHINE_DISCHARGING:
        if (due(core, now_ms, core->cycle.discharge_ms)) {
            begin_close(core, now_ms);
        }
        break;
    case MACHINE_CLOSE_DOOR:
        if (due(core, now_ms, core->cycle.door_close_ms)) {
            begin_runon(core, now_ms);
        }
        break;
    case MACHINE_MOTOR_RUNON: {
        uint32_t elapsed = now_ms - core->state_entered_ms;
        if (elapsed >= core->cycle.shredder_run_on_ms) {
            core->status.outputs.shredder = false;
        }
        if (elapsed >= core->cycle.grinder_run_on_ms) {
            core->status.outputs.grinder = false;
        }
        if (!core->status.outputs.shredder && !core->status.outputs.grinder) {
            core->status.outputs.door_open = false;
            enter_state(core, MACHINE_BATCH_COMPLETE, now_ms);
        }
        break;
    }
    case MACHINE_BATCH_COMPLETE:
        if (rearm_ready(core, scale)) {
            outputs_off(core);
            core->status.batch_armed = true;
            core->status.scale_warning = false;
            core->status.fault = FAULT_NONE;
            clear_window(core);
            enter_state(core, MACHINE_IDLE, now_ms);
        }
        break;
    case MACHINE_MANUAL:
    case MACHINE_FAULT:
        break;
    }
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure -R test_machine_core`

Expected: `100% tests passed` and `ok`. If a timing expectation is off by one 10 ms step, fix the test cursor or the comparison so the asserted elapsed time is the timestamp passed into the transitioning step. Do not weaken a transition so two states change in one call.

- [ ] **Step 5: Commit**

```bash
git add main/machine/machine_core.c test/host/test_machine_core.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add the batch state machine with host-tested arming and fault rules.

EOF
)"
```

### Task 4: Modbus master, scale decode, and device profile

**Files:**
- Create: `main/modbus/modbus_master.h`
- Create: `main/modbus/modbus_master.c`
- Create: `main/device/scale_decode.h`
- Create: `main/device/scale_decode.c`
- Create: `main/device/profile.h`
- Create: `main/device/profile.c`
- Create: `test/host/test_modbus_master.c`
- Create: `test/host/test_scale_decode.c`
- Modify: `test/host/CMakeLists.txt`
- Test: `test/host/test_modbus_master.c`, `test/host/test_scale_decode.c`

**Interfaces:**
- Consumes: `crc16_modbus`
- Produces:
  - `modbus_status_t` with the nine codes named in the spec
  - `modbus_link_t` with `write` returning 0 and `read` returning a byte count, 0, or negative
  - `modbus_status_t modbus_read_registers(const modbus_link_t *link, uint8_t slave, uint8_t function, uint16_t address, uint16_t count, uint16_t *regs);`
  - `modbus_status_t modbus_write_coils(const modbus_link_t *link, uint8_t slave, uint16_t address, uint16_t count, uint8_t coil_byte);`
  - `modbus_status_t modbus_write_single_coil(const modbus_link_t *link, uint8_t slave, uint16_t address, bool on);`
  - `bool scale_decode(scale_decode_t mode, const uint16_t *regs, size_t count, int32_t *grams);`
  - `const device_profile_t *device_profile(void);` returning the initial profile from the spec

- [ ] **Step 1: Write the failing tests**

Add these sources to `firmware_host` and register both test executables:

```cmake
    ${ROOT}/main/modbus/modbus_master.c
    ${ROOT}/main/device/scale_decode.c
    ${ROOT}/main/device/profile.c
```

```cmake
add_host_test(test_modbus_master)
add_host_test(test_scale_decode)
```

Create `test/host/test_modbus_master.c` with a fake transport that records the last TX frame and returns a prepared RX buffer. Cover:

- A read of slave 1, function `0x03`, address 0, count 2 sends `01 03 00 00 00 02` plus the CRC from `crc16_modbus`. The reply `01 03 04 00 00 67 20 D1 DB` returns `MODBUS_OK` and registers `{0x0000, 0x6720}`.
- The same reply with the last byte flipped returns `MODBUS_ERR_CRC`.
- An empty read returns `MODBUS_ERR_TIMEOUT`.
- Reply `01 83 02 C0 F1` returns `MODBUS_ERR_EXCEPTION`.
- `modbus_write_coils(link, 2, 0, 3, 0x07)` sends `02 0F 00 00 00 03 01 07` plus CRC. Reply `02 0F 00 00 00 03 15 F9` returns `MODBUS_OK`.
- `modbus_write_single_coil(link, 2, 0, true)` sends `02 05 00 00 FF 00` plus CRC. The matching 8-byte echo returns `MODBUS_OK`.
- `count == 0` returns `MODBUS_ERR_ARGUMENT`.
- A write callback that returns -1 produces `MODBUS_ERR_TRANSPORT`.

Create `test/host/test_scale_decode.c`:

- `SCALE_DECODE_S32_BE_GRAMS` of `{0x0000, 0x6720}` is 26400.
- `SCALE_DECODE_S32_LE_GRAMS` of `{0x6720, 0x0000}` is 26400.
- `SCALE_DECODE_S32_BE_GRAMS` of `{0xFFFF, 0xFFFF}` is -1.
- `SCALE_DECODE_S16_GRAMS` of `{0xFF9C}` with count 1 is -100.
- `SCALE_DECODE_S16_GRAMS` with count 2 returns false.
- `device_profile()` matches the spec: baud 9600, TX 17, RX 18, DE 16, `use_de` true, parity none, UART port 1, scale slave 1 function `0x03` register 0 count 2 decode `SCALE_DECODE_S32_BE_GRAMS`, I/O slave 2 function `0x0F` coil 0 count 3, panel 320 by 240, SCLK 4, MOSI 5, MISO 6, CS 7, DC 8, RST 9, backlight 10 active high, `swap_xy` true, mirrors false, SPI host value 2, encoder A 11 B 12 switch 13 active low, poll 100, refresh 500, stale 500, timeout 100, control period 10, long press 800, button debounce 20, quadrature debounce 2, steps per detent 4.

Build the expected CRC in the test with `crc16_modbus` for the request frames. Use the hardcoded reply bytes above so a broken CRC check fails.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: failure text contains `modbus_master.c`.

- [ ] **Step 3: Write the minimal implementation**

`modbus_read_registers` rejects a null link, a null output, `count == 0`, `count > 8`, and a function other than `0x03` or `0x04` with `MODBUS_ERR_ARGUMENT`. It writes the 8-byte request, CRC low byte first. `write` returning non-zero is `MODBUS_ERR_TRANSPORT`. `read` returning negative is `MODBUS_ERR_TRANSPORT`, 0 is `MODBUS_ERR_TIMEOUT`, and fewer than 5 bytes is `MODBUS_ERR_LENGTH`. CRC covers every byte except the last two. Then check slave, exception bit, function, byte count `count * 2`, and total length `5 + count * 2`. Register words are big-endian.

`modbus_write_coils` sends function `0x0F` with one data byte and expects an 8-byte echo of slave, function, address, and quantity. `modbus_write_single_coil` sends function `0x05` with `0xFF00` for on and `0x0000` for off, and expects the same 8 bytes back. A mismatched echo is `MODBUS_ERR_FUNCTION`.

`scale_decode` implements the three modes from the spec. Any other mode, a null pointer, or the wrong count returns false.

`device_profile()` returns a static `device_profile_t` with the values in the test list. `spi_host == 2` means SPI2; `board.c` maps that to `SPI2_HOST`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure -R "test_modbus_master|test_scale_decode"`

Expected: `100% tests passed`.

- [ ] **Step 5: Commit**

```bash
git add main/modbus main/device test/host/test_modbus_master.c test/host/test_scale_decode.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add the RTU master and the compiled scale and pin profile.

EOF
)"
```

### Task 5: Encoder decoder

**Files:**
- Create: `main/ui/encoder_decode.h`
- Create: `main/ui/encoder_decode.c`
- Create: `test/host/test_encoder_decode.c`
- Modify: `test/host/CMakeLists.txt`
- Test: `test/host/test_encoder_decode.c`

**Interfaces:**
- Consumes: nothing
- Produces:
  - `enc_event_t` with `type` of `ENC_ROTATE`, `ENC_SHORT`, or `ENC_LONG`, and `int8_t steps`
  - `void encoder_decode_init(encoder_decode_t *dec);`
  - `bool encoder_decode_step(encoder_decode_t *dec, uint32_t now_ms, uint8_t a, uint8_t b, uint8_t sw, enc_event_t *out);`
  - The first sample is the baseline and emits nothing. One detent is four quadrature transitions. The table for index `(old_ab << 2) | new_ab` is `{0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0}` where `ab = (a << 1) | b`. Clockwise `00,01,11,10,00` emits one `ENC_ROTATE` with `steps == 1`. The opposite sequence emits `steps == -1`.
  - Switch low is pressed. A release before 800 ms emits `ENC_SHORT`. Holding to 800 ms emits `ENC_LONG` once and the release emits nothing. Edges closer than 20 ms are ignored. Quadrature edges closer than 2 ms are ignored. At most one event is returned per call; a second event from the same sample is returned on the next call.

- [ ] **Step 1: Write the failing test**

Add `encoder_decode.c` to the host library and `add_host_test(test_encoder_decode)`.

The test feeds the clockwise and counterclockwise sequences at 5 ms spacing, the 800 ms long press, a 100 ms short press, and a release 10 ms after the press that must not emit until 20 ms have passed. Call `encoder_decode_step` in a loop when a sample can produce two events.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: failure text contains `encoder_decode.c`.

- [ ] **Step 3: Write the minimal implementation**

Implement the header and `.c` file to the interface above. Quadrature state and the pending event live in `encoder_decode_t`. Do not include ESP-IDF headers.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure -R test_encoder_decode`

Expected: `100% tests passed`.

- [ ] **Step 5: Commit**

```bash
git add main/ui/encoder_decode.h main/ui/encoder_decode.c test/host/test_encoder_decode.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add detent and press decoding for the panel encoder.

EOF
)"
```

### Task 6: UI navigation logic

**Files:**
- Create: `main/ui/ui_logic.h`
- Create: `main/ui/ui_logic.c`
- Create: `test/host/test_ui_logic.c`
- Modify: `test/host/CMakeLists.txt`
- Test: `test/host/test_ui_logic.c`

**Interfaces:**
- Consumes: `machine_settings_valid`, `machine_settings_defaults`, `enc_event_t`
- Produces:
  - Screens `UI_SCREEN_MAIN`, `UI_SCREEN_MENU`, `UI_SCREEN_STOP_CONFIRM`, `UI_SCREEN_SETTINGS`, `UI_SCREEN_MANUAL_CONFIRM`, `UI_SCREEN_MANUAL`, `UI_SCREEN_DIAGNOSTICS`
  - `void ui_init(ui_model_t *ui, const machine_settings_t *settings);`
  - `void ui_on_encoder(ui_model_t *ui, const ui_view_t *view, const enc_event_t *event, operator_event_t *command);`
  - `bool ui_start_allowed(const ui_model_t *ui);` true only on `UI_SCREEN_MAIN`
  - `void ui_on_settings_saved(ui_model_t *ui, bool ok);`
  - `void ui_format_weight(int32_t grams, char *out, size_t out_len);` — `25000` formats as `25.00 kg`, `-150` as `-0.15 kg`
  - `void ui_format_time(uint32_t ms, char *out, size_t out_len);` — `2000` formats as `2.0 s`
  - `const char *ui_setting_label(int index);` for the twelve names in the spec, then `Save` and `Back`
  - Setting ranges and steps from the spec table. An edit that leaves the range or fails `machine_settings_valid` is discarded.

`ui_view_t` carries `machine_state_t state` and `machine_settings_t settings`. `ui_on_encoder` writes exactly one command; unused results are `CMD_NONE`.

Navigation rules are the operator-interface section of the spec. A fault-state short press sends `CMD_ACK_FAULT` from any screen. Save sets `ui->save_requested` and stays on settings until `ui_on_settings_saved`. Success copies the edited settings into the saved copy and returns to the menu. Failure stays on settings. Back restores the copy captured when settings was opened.

- [ ] **Step 1: Write the failing test**

Add `ui_logic.c` to the host library and `add_host_test(test_ui_logic)`.

Tests:

- Long press on main while `MACHINE_IDLE` opens the menu and emits `CMD_NONE`. `ui_start_allowed` is false on the menu.
- Short press on menu row 0 opens settings. Rotate onto minimum batch, edit it upward by one step, short press to leave edit. The in-screen value is 1100 and the saved copy is still 1000.
- Short press on Back restores 1000 and returns to the menu.
- Raising reset threshold to the target is ignored.
- Long press on main while `MACHINE_DISCHARGING` opens stop confirm. Short press on Stop emits `CMD_STOP_CONFIRMED` and returns to main.
- Short press on Cancel emits `CMD_NONE` and returns to main.
- Menu row Manual opens confirm. Enable emits `CMD_ENTER_MANUAL`. A short press on the shredder row emits `CMD_SET_SHREDDER` with `level == true`. Back emits `CMD_EXIT_MANUAL`.
- While `view.state == MACHINE_FAULT`, a short press emits `CMD_ACK_FAULT` and a long press does not open the menu.
- `ui_format_weight(25000)` is `25.00 kg`. `ui_format_time(2000)` is `2.0 s`.
- Save sets `save_requested`. `ui_on_settings_saved(ui, true)` clears it and lands on the menu. `ui_on_settings_saved(ui, false)` stays on settings.

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S test/host -B build/host`

Expected: failure text contains `ui_logic.c`.

- [ ] **Step 3: Write the minimal implementation**

Implement `ui_logic.h` and `ui_logic.c` with no ESP-IDF includes. Menu rows are Settings, Manual, Diagnostics. Stop rows are Stop, Cancel. Manual confirm rows are Enable, Cancel. Manual rows are Shredder, Grinder, Door, Back. Diagnostics short press returns to the menu. Rotate wraps the current list, including a negative step.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake -S test/host -B build/host && cmake --build build/host --config Debug && ctest --test-dir build/host -C Debug --output-on-failure`

Expected: `100% tests passed` for every host test registered so far.

- [ ] **Step 5: Commit**

```bash
git add main/ui/ui_logic.h main/ui/ui_logic.c test/host/test_ui_logic.c test/host/CMakeLists.txt
git commit -m "$(cat <<'EOF'
Add encoder navigation for stop, settings, and manual mode.

EOF
)"
```

### Task 7: Flashable controller without the panel

**Files:**
- Create: `CMakeLists.txt`
- Create: `sdkconfig.defaults`
- Create: `partitions.csv`
- Create: `.gitignore`
- Create: `main/CMakeLists.txt`
- Create: `main/idf_component.yml`
- Create: `main/main.c`
- Create: `main/status/status_bus.h`
- Create: `main/status/status_bus.c`
- Create: `main/settings/settings_store.h`
- Create: `main/settings/settings_store.c`
- Create: `main/device/board.h`
- Create: `main/device/board.c`
- Create: `main/machine/control_task.c`
- Create: `main/modbus/modbus_task.c`
- Test: `idf.py build` and the existing host suite

**Interfaces:**
- Consumes: the host-tested core, codec, master, profile, and scale decode
- Produces: `status_bus_*` accessors, `settings_store_load`, `settings_store_save`, `board_modbus_init`, `board_modbus_link`, `control_task`, and `modbus_task`. Tasks are created with the priorities, CPUs, and stack sizes in the spec. This task does not create the HMI task; `start_allowed` stays at its initial true value so the sequence can run during bus bring-up.

- [ ] **Step 1: Add the project files**

Root `CMakeLists.txt` is build system v1:

```cmake
cmake_minimum_required(VERSION 3.22)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(fish_shredder)
```

`sdkconfig.defaults`:

```text
CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y
CONFIG_ESPTOOLPY_FLASHMODE_QIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_FREERTOS_HZ=1000
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
```

`partitions.csv`:

```text
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x300000,
```

`.gitignore`:

```text
build/
sdkconfig
sdkconfig.old
managed_components/
```

`main/idf_component.yml`:

```yaml
dependencies:
  idf:
    version: ">=6.1.0,<6.2.0"
```

LVGL is added in Task 8. Do not add it here.

`settings_store_load` opens namespace `machine`, reads blob `cfg`, and unpacks it. Any failure returns false and writes `machine_settings_defaults()` to the out pointer. `settings_store_save` packs 54 bytes, `nvs_set_blob`, and `nvs_commit`.

`status_bus_init` creates a mutex and an 8-deep queue of `operator_event_t`. Initial machine status is idle with outputs off. Initial I/O snapshot has `link_ok` true and `attempted` false. Initial scale snapshot has `last_ok` false and `have_sample` false. Settings are the defaults. `start_allowed` is true. Both settings flags are false. Get and set copy under the mutex. `status_bus_send_command` and `status_bus_take_command` are non-blocking.

`board_modbus_init` installs UART port 1 at the profile baud, 8 data bits, the profile parity, and one stop bit. TX, RX, and DE pins come from the profile. `use_de` true selects `UART_MODE_RS485_HALF_DUPLEX` with DE on the UART RTS pin. `use_de` false selects `UART_MODE_UART` and no RTS pin. `board_write` writes the bytes, waits for TX done, flushes RX, and returns 0. `board_read` waits up to the requested timeout for the first byte, then reads until a 5 ms gap or 64 bytes. It returns the byte count, 0 on a first-byte timeout, and -1 on a driver error.

`modbus_task` polls the scale every `scale_poll_ms`. On `MODBUS_OK` and a successful `scale_decode`, it stores grams, sets `last_ok` and `have_sample`, and sets `sample_ms`. On failure it sets `last_ok` false and leaves the previous grams. It writes coils when the output image changes and every `output_refresh_ms`. Coil bit 0 is the shredder, bit 1 the grinder, bit 2 the door. Function `0x0F` uses `modbus_write_coils`. Function `0x05` writes the three coils with `modbus_write_single_coil` and reports the first failure. `link_ok` follows that result and `attempted` becomes true. Failures log with `ESP_LOGW` on tag `modbus`.

`control_task` wakes every `control_period_ms`. It publishes settings into the core, builds `scale_sample_t.valid` from `last_ok && grams >= 0 && age <= scale_stale_ms`, takes one command, reads `start_allowed` and `io.link_ok`, calls `machine_core_step`, logs state changes with `ESP_LOGI` on tag `control` and `machine_state_name`, and publishes the status. The clock is `(uint32_t)(esp_timer_get_time() / 1000)`.

`app_main` calls `nvs_flash_init`, erases and retries when the error is `ESP_ERR_NVS_NO_FREE_PAGES` or `ESP_ERR_NVS_NEW_VERSION_FOUND`, loads settings, initializes the status bus, publishes settings and `settings_reset`, initializes the UART, then creates `modbus_task` at priority 9 on CPU 1 and `control_task` at priority 10 on CPU 1. Both stacks are 4096.

`main/CMakeLists.txt` registers every `.c` file created through this task. `REQUIRES` includes `esp_driver_uart`, `nvs_flash`, and `esp_timer`.

- [ ] **Step 2: Build the firmware and rerun the host tests**

From an ESP-IDF v6.1.1 shell at the repo root:

```powershell
idf.py set-target esp32s3
idf.py build
cmake -S test/host -B build/host
cmake --build build/host --config Debug
ctest --test-dir build/host -C Debug --output-on-failure
```

Expected: the firmware log contains `Project build complete`. Host tests print `100% tests passed`.

`idf.py set-target esp32s3` rewrites `sdkconfig`. Keep `sdkconfig.defaults` as the committed source of the chip options. Do not commit `sdkconfig`.

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt sdkconfig.defaults partitions.csv .gitignore main
git commit -m "$(cat <<'EOF'
Boot a Modbus controller that runs the batch sequence without a panel.

EOF
)"
```

### Task 8: Panel, encoder, and HMI task

**Files:**
- Create: `main/hmi/display.h`
- Create: `main/hmi/display.c`
- Create: `main/hmi/encoder_hw.h`
- Create: `main/hmi/encoder_hw.c`
- Create: `main/hmi/hmi_task.c`
- Modify: `main/main.c`
- Modify: `main/CMakeLists.txt`
- Modify: `main/idf_component.yml`
- Modify: `sdkconfig.defaults`
- Test: `idf.py build`

**Interfaces:**
- Consumes: `ui_logic`, `encoder_decode`, `device_profile`, `status_bus`, `settings_store`
- Produces: `display_init`, `encoder_hw_init`, `encoder_hw_take`, `encoder_hw_latest`, and `hmi_task`. After this task, `app_main` creates the HMI task first, then Modbus, then control, which is the spec boot order.

- [ ] **Step 1: Add the panel and the HMI task**

Append to `main/idf_component.yml`:

```yaml
  lvgl/lvgl: "9.5.0"
  espressif/esp_lcd_ili9341: "2.1.0"
```

Append to `sdkconfig.defaults`:

```text
CONFIG_LV_FONT_MONTSERRAT_14=y
CONFIG_LV_FONT_MONTSERRAT_20=y
CONFIG_LV_COLOR_DEPTH_16=y
```

`display_init` initializes SPI2 from `panel.spi_host == 2`, adds the ILI9341 panel on the profile pins at 20 MHz, SPI mode 0, 16 bits per pixel, and `LCD_RGB_ELEMENT_ORDER_BGR`. It applies `swap_xy`, `mirror_x`, and `mirror_y`, turns the display on, and drives the backlight from `bl_active_high`. It calls `lv_init`, creates a 320 by 240 display, and sets a partial RGB565 flush. The flush callback calls `esp_lcd_panel_draw_bitmap` and does not call `lv_display_flush_ready`. The panel I/O `on_color_trans_done` callback calls `lv_display_flush_ready`. Each draw buffer is `h_res * 24` RGB565 pixels from `heap_caps_malloc` with `MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`.

`encoder_hw_init` configures the three encoder GPIOs as inputs with pull-ups and `GPIO_INTR_ANYEDGE`. The ISR reads A, B, and the switch, stores them as the latest sample under a port mux, and posts that sample to a 32-deep queue. `encoder_hw_take` is non-blocking. `encoder_hw_latest` returns the mux-protected sample.

`hmi_task` initializes `ui_model_t` from the published settings and `encoder_decode_t`. Every 10 ms it drains encoder samples through `encoder_decode_step`, then calls the decoder once more with the latest levels and the current millisecond time so a held button can reach 800 ms. Each decoded event calls `ui_on_encoder` with a view built from the machine status and the published settings. A command other than `CMD_NONE` is queued. When `save_requested` is set, `settings_store_save` runs. Success publishes the settings and clears both settings flags. Failure sets `settings_save_failed` and logs `ESP_LOGE` on tag `hmi`. Then `ui_on_settings_saved` runs. The task writes `ui_start_allowed` into the status bus, updates the labels below, and calls `lv_timer_handler`.

Labels, in Montserrat 20 for the weight and Montserrat 14 for the rest:

- Weight from the scale snapshot, or `--` when `have_sample` is false
- Target from the published settings
- `Stable` or `Unstable` from machine status
- State name
- Shredder and grinder `On` or `Off`
- `Door command` `Open` or `Close`
- Scale link text and I/O link text from the spec tables
- Banner `Scale lost during batch`, `Settings reset to defaults`, and `Settings were not saved` when those flags are set

While the machine state is `MACHINE_FAULT`, show `I/O link lost` and hide the other screens. Otherwise show the container for `ui->screen`. Menu, settings, stop confirm, manual confirm, manual, and diagnostics render their current row and mark the selected row. Settings shows `ui_setting_label` and the formatted value. Diagnostics adds baud, both slave IDs, the scale register, the decode name (`S32_BE_GRAMS`, `S32_LE_GRAMS`, or `S16_GRAMS`), and the last Modbus status codes.

Add the three new `.c` files to `main/CMakeLists.txt`. Add `esp_lcd`, `esp_driver_gpio`, `esp_driver_spi`, `esp_lcd_ili9341`, and `lvgl` to `REQUIRES`.

Replace the task-creation section of `app_main` so the order is `hmi_task` (priority 5, CPU 0, stack 8192), `modbus_task`, `control_task`. Call `display_init` and `encoder_hw_init` before the tasks start.

- [ ] **Step 2: Build**

```powershell
idf.py build
cmake --build build/host --config Debug
ctest --test-dir build/host -C Debug --output-on-failure
```

Expected: `Project build complete` and `100% tests passed`. If a LVGL Kconfig symbol name differs in 9.5.0, change `sdkconfig.defaults` until `lv_font_montserrat_14` and `lv_font_montserrat_20` link. Do not switch the project to build system v2 to get past a PSRAM error.

- [ ] **Step 3: Commit**

```bash
git add main sdkconfig.defaults
git commit -m "$(cat <<'EOF'
Show machine status on the panel and accept encoder commands.

EOF
)"
```

Commit `dependencies.lock` in this same commit if `idf.py build` created it.

## Plan self-review

Spec coverage is mapped as follows.

| Spec section | Task |
|---|---|
| CRC, settings record, defaults, ranges | Tasks 1, 2, 6 |
| Stability, arming, one-shot batch, cycle timers | Task 3 |
| State list, stop, fault, manual, scale-loss rules | Task 3 |
| Modbus status codes, frames, scale decode, initial profile | Task 4 |
| Encoder gestures | Tasks 5 and 8 |
| Screens, banners, formats, `start_allowed` | Tasks 6 and 8 |
| 10 ms control, 100 ms poll, 500 ms rewrite, stale sample | Task 7 |
| NVS load and save, boot order, three tasks, LVGL flush | Tasks 7 and 8 |
| Build system v1 and octal PSRAM | Task 7 |

Host tests do not flash the board. Live scale, contactor, and actuator checks stay in the hardware commissioning stages.