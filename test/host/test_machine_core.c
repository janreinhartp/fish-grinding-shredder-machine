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
