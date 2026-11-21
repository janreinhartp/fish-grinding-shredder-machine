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
