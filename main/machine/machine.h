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
