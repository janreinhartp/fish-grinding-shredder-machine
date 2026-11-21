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
