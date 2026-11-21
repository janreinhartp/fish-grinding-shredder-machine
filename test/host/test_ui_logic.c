#include <string.h>

#include "machine/machine.h"
#include "test_harness.h"
#include "ui/encoder_decode.h"
#include "ui/ui_logic.h"

int g_failures = 0;

static const enc_event_t SHORT_PRESS = {ENC_SHORT, 0};
static const enc_event_t LONG_PRESS = {ENC_LONG, 0};

static enc_event_t rotate(int8_t steps) {
    enc_event_t e = {ENC_ROTATE, steps};
    return e;
}

static operator_event_t send(ui_model_t *ui, machine_state_t state, enc_event_t ev) {
    ui_view_t view;
    operator_event_t cmd = {CMD_STOP_CONFIRMED, true}; /* poisoned: must be overwritten */
    view.state = state;
    view.settings = machine_settings_defaults();
    ui_on_encoder(ui, &view, &ev, &cmd);
    return cmd;
}

static void new_ui(ui_model_t *ui) {
    machine_settings_t s = machine_settings_defaults();
    ui_init(ui, &s);
}

static void test_init(void) {
    ui_model_t ui;
    new_ui(&ui);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
    EXPECT_EQ_INT(ui.save_requested, 0);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1000);
    EXPECT_EQ_INT(ui.saved.minimum_batch_g, 1000);
    EXPECT(ui_start_allowed(&ui));
}

static void test_long_press_idle_opens_menu(void) {
    ui_model_t ui;
    new_ui(&ui);
    operator_event_t cmd = send(&ui, MACHINE_IDLE, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT(!ui_start_allowed(&ui));
}

static void test_settings_edit_and_back(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    operator_event_t cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS); /* row 0: Settings */
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_SETTINGS);
    EXPECT(!ui_start_allowed(&ui));

    send(&ui, MACHINE_IDLE, rotate(1)); /* target_g -> minimum_batch_g */
    EXPECT_EQ_INT(ui.row, 1);
    send(&ui, MACHINE_IDLE, SHORT_PRESS); /* arm edit */
    EXPECT_EQ_INT(ui.editing, 1);
    send(&ui, MACHINE_IDLE, rotate(1));
    send(&ui, MACHINE_IDLE, SHORT_PRESS); /* leave edit */
    EXPECT_EQ_INT(ui.editing, 0);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1100);
    EXPECT_EQ_INT(ui.saved.minimum_batch_g, 1000);

    send(&ui, MACHINE_IDLE, rotate(-1)); /* row 0 */
    send(&ui, MACHINE_IDLE, rotate(-1)); /* wraps to Back */
    EXPECT_EQ_INT(ui.row, UI_SETTINGS_ROW_BACK);
    cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1000);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT_EQ_INT(ui.row, 0);
}

static void test_edit_clamps_to_range_and_steps(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    /* tolerance_g is row 2: step 10, minimum 10. */
    send(&ui, MACHINE_IDLE, rotate(2));
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1));
    EXPECT_EQ_INT(ui.settings.tolerance_g, 110);
    send(&ui, MACHINE_IDLE, rotate(-1));
    send(&ui, MACHINE_IDLE, rotate(-1));
    EXPECT_EQ_INT(ui.settings.tolerance_g, 90);
    for (int i = 0; i < 20; i++) {
        send(&ui, MACHINE_IDLE, rotate(-1));
    }
    EXPECT_EQ_INT(ui.settings.tolerance_g, 10); /* below the minimum is discarded */
    /* Several detents in one event are applied one step at a time. */
    send(&ui, MACHINE_IDLE, rotate(3));
    EXPECT_EQ_INT(ui.settings.tolerance_g, 40);
}

static void test_reset_threshold_to_target_is_ignored(void) {
    ui_model_t ui;
    new_ui(&ui);
    ui.saved.reset_threshold_g = 24900; /* settings opens from the saved copy */
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, rotate(4)); /* reset_threshold_g */
    EXPECT_EQ_INT(ui.row, 4);
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1)); /* 25000 == target: invalid */
    EXPECT_EQ_INT(ui.settings.reset_threshold_g, 24900);
    send(&ui, MACHINE_IDLE, rotate(-1));
    EXPECT_EQ_INT(ui.settings.reset_threshold_g, 24800);
}

static void test_stop_confirm(void) {
    ui_model_t ui;
    new_ui(&ui);
    operator_event_t cmd = send(&ui, MACHINE_DISCHARGING, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_STOP_CONFIRM);
    EXPECT(!ui_start_allowed(&ui));
    cmd = send(&ui, MACHINE_DISCHARGING, SHORT_PRESS); /* row 0: Stop */
    EXPECT_EQ_INT(cmd.type, CMD_STOP_CONFIRMED);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);

    send(&ui, MACHINE_DISCHARGING, LONG_PRESS);
    send(&ui, MACHINE_DISCHARGING, rotate(1)); /* Cancel */
    cmd = send(&ui, MACHINE_DISCHARGING, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
}

static void test_manual_mode(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1)); /* Manual */
    operator_event_t cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MANUAL_CONFIRM);

    /* Cancel returns to the menu without a command. */
    send(&ui, MACHINE_IDLE, rotate(1));
    cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);

    send(&ui, MACHINE_IDLE, SHORT_PRESS); /* menu row is still Manual */
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MANUAL_CONFIRM);
    cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS); /* Enable */
    EXPECT_EQ_INT(cmd.type, CMD_ENTER_MANUAL);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MANUAL);

    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS); /* Shredder */
    EXPECT_EQ_INT(cmd.type, CMD_SET_SHREDDER);
    EXPECT_EQ_INT(cmd.level, 1);
    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_SET_SHREDDER);
    EXPECT_EQ_INT(cmd.level, 0);
    send(&ui, MACHINE_MANUAL, rotate(1));
    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_SET_GRINDER);
    EXPECT_EQ_INT(cmd.level, 1);
    send(&ui, MACHINE_MANUAL, rotate(1));
    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_SET_DOOR);
    EXPECT_EQ_INT(cmd.level, 1);

    /* A long press in manual is ignored. */
    cmd = send(&ui, MACHINE_MANUAL, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MANUAL);

    send(&ui, MACHINE_MANUAL, rotate(1)); /* Back */
    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_EXIT_MANUAL);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
    EXPECT(ui_start_allowed(&ui));
}

static void test_fault(void) {
    ui_model_t ui;
    new_ui(&ui);
    operator_event_t cmd = send(&ui, MACHINE_FAULT, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
    cmd = send(&ui, MACHINE_FAULT, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_ACK_FAULT);

    /* Acknowledge works from any screen. */
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    cmd = send(&ui, MACHINE_FAULT, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_ACK_FAULT);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
}

static void test_fault_ack_resets_manual_screen(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1)); /* Manual */
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    operator_event_t cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS); /* Enable */
    EXPECT_EQ_INT(cmd.type, CMD_ENTER_MANUAL);
    cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS); /* Shredder on */
    EXPECT_EQ_INT(cmd.type, CMD_SET_SHREDDER);
    EXPECT_EQ_INT(ui.manual_shredder, 1);

    /* Rotate and long press in fault do nothing and keep the screen. */
    cmd = send(&ui, MACHINE_FAULT, rotate(1));
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    cmd = send(&ui, MACHINE_FAULT, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MANUAL);

    cmd = send(&ui, MACHINE_FAULT, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_ACK_FAULT);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
    EXPECT_EQ_INT(ui.editing, 0);
    EXPECT_EQ_INT(ui.manual_shredder, 0);
    EXPECT_EQ_INT(ui.manual_grinder, 0);
    EXPECT_EQ_INT(ui.manual_door, 0);
}

static void test_menu_settings_rejected_after_batch_started(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS); /* menu opened from a snapshot that is now stale */
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    operator_event_t cmd = send(&ui, MACHINE_START_GRINDER, SHORT_PRESS); /* row 0: Settings */
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT_EQ_INT(ui.row, 0);
}

static void test_menu_manual_rejected_after_batch_started(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1)); /* Manual */
    operator_event_t cmd = send(&ui, MACHINE_WEIGHT_VALIDATION, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT_EQ_INT(ui.row, 1);
}

static void test_manual_screen_returns_to_main_when_not_in_manual(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1));
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, SHORT_PRESS); /* Enable */
    operator_event_t cmd = send(&ui, MACHINE_MANUAL, SHORT_PRESS); /* Shredder on */
    EXPECT_EQ_INT(cmd.type, CMD_SET_SHREDDER);
    EXPECT_EQ_INT(ui.manual_shredder, 1);

    /* The machine is no longer in manual: no SET command may be sent. */
    cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MAIN);
    EXPECT_EQ_INT(ui.manual_shredder, 0);
    EXPECT_EQ_INT(ui.manual_grinder, 0);
    EXPECT_EQ_INT(ui.manual_door, 0);
}

static void test_other_screens_ignore_long_press(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    operator_event_t cmd = send(&ui, MACHINE_IDLE, LONG_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
}

static void test_diagnostics(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(2));
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_DIAGNOSTICS);
    EXPECT(!ui_start_allowed(&ui));
    operator_event_t cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
}

static void test_menu_wraps(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, rotate(-1));
    EXPECT_EQ_INT(ui.row, 2);
    send(&ui, MACHINE_IDLE, rotate(1));
    EXPECT_EQ_INT(ui.row, 0);
    send(&ui, MACHINE_IDLE, rotate(-4));
    EXPECT_EQ_INT(ui.row, 2);
}

static void test_save_flow(void) {
    ui_model_t ui;
    new_ui(&ui);
    send(&ui, MACHINE_IDLE, LONG_PRESS);
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1));
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    send(&ui, MACHINE_IDLE, rotate(1));
    send(&ui, MACHINE_IDLE, SHORT_PRESS); /* minimum_batch_g = 1100 */
    send(&ui, MACHINE_IDLE, rotate(-3));  /* row 1 wraps to Save */
    EXPECT_EQ_INT(ui.row, UI_SETTINGS_ROW_SAVE);
    operator_event_t cmd = send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(cmd.type, CMD_NONE);
    EXPECT_EQ_INT(ui.save_requested, 1);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_SETTINGS);

    ui_on_settings_saved(&ui, false);
    EXPECT_EQ_INT(ui.save_requested, 0);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_SETTINGS);
    EXPECT_EQ_INT(ui.saved.minimum_batch_g, 1000);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1100);

    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(ui.save_requested, 1);
    ui_on_settings_saved(&ui, true);
    EXPECT_EQ_INT(ui.save_requested, 0);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT_EQ_INT(ui.saved.minimum_batch_g, 1100);

    /* Re-opening settings and pressing Back keeps the saved value. */
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_SETTINGS);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1100);
    send(&ui, MACHINE_IDLE, rotate(-1));
    send(&ui, MACHINE_IDLE, SHORT_PRESS);
    EXPECT_EQ_INT(ui.screen, UI_SCREEN_MENU);
    EXPECT_EQ_INT(ui.settings.minimum_batch_g, 1100);
}

static void test_format(void) {
    char buf[32];
    ui_format_weight(25000, buf, sizeof buf);
    EXPECT(strcmp(buf, "25.00 kg") == 0);
    ui_format_weight(-150, buf, sizeof buf);
    EXPECT(strcmp(buf, "-0.15 kg") == 0);
    ui_format_weight(0, buf, sizeof buf);
    EXPECT(strcmp(buf, "0.00 kg") == 0);
    /* Rounds to the nearest hundredth; never prints a negative zero. */
    ui_format_weight(4, buf, sizeof buf);
    EXPECT(strcmp(buf, "0.00 kg") == 0);
    ui_format_weight(5, buf, sizeof buf);
    EXPECT(strcmp(buf, "0.01 kg") == 0);
    ui_format_weight(-4, buf, sizeof buf);
    EXPECT(strcmp(buf, "0.00 kg") == 0);
    ui_format_weight(INT32_MIN, buf, sizeof buf);
    EXPECT(strcmp(buf, "-2147483.65 kg") == 0);
    ui_format_time(UINT32_MAX, buf, sizeof buf);
    EXPECT(strcmp(buf, "4294967.3 s") == 0);
    ui_format_weight(-1500, buf, sizeof buf);
    EXPECT(strcmp(buf, "-1.50 kg") == 0);
    ui_format_time(2000, buf, sizeof buf);
    EXPECT(strcmp(buf, "2.0 s") == 0);
    ui_format_time(120000, buf, sizeof buf);
    EXPECT(strcmp(buf, "120.0 s") == 0);
    ui_format_time(100, buf, sizeof buf);
    EXPECT(strcmp(buf, "0.1 s") == 0);
    /* Truncates safely into a small buffer. */
    char small[4] = "xxx";
    ui_format_weight(25000, small, sizeof small);
    EXPECT_EQ_INT(small[3], 0);
}

static void test_labels(void) {
    EXPECT(strcmp(ui_setting_label(0), "target_g") == 0);
    EXPECT(strcmp(ui_setting_label(1), "minimum_batch_g") == 0);
    EXPECT(strcmp(ui_setting_label(2), "tolerance_g") == 0);
    EXPECT(strcmp(ui_setting_label(3), "stable_ms") == 0);
    EXPECT(strcmp(ui_setting_label(4), "reset_threshold_g") == 0);
    EXPECT(strcmp(ui_setting_label(5), "grinder_startup_ms") == 0);
    EXPECT(strcmp(ui_setting_label(6), "shredder_startup_ms") == 0);
    EXPECT(strcmp(ui_setting_label(7), "door_open_ms") == 0);
    EXPECT(strcmp(ui_setting_label(8), "discharge_ms") == 0);
    EXPECT(strcmp(ui_setting_label(9), "door_close_ms") == 0);
    EXPECT(strcmp(ui_setting_label(10), "shredder_run_on_ms") == 0);
    EXPECT(strcmp(ui_setting_label(11), "grinder_run_on_ms") == 0);
    EXPECT(strcmp(ui_setting_label(12), "Save") == 0);
    EXPECT(strcmp(ui_setting_label(13), "Back") == 0);
    EXPECT(ui_setting_label(-1) != NULL);
    EXPECT(ui_setting_label(14) != NULL);
}

int main(void) {
    test_init();
    test_long_press_idle_opens_menu();
    test_settings_edit_and_back();
    test_edit_clamps_to_range_and_steps();
    test_reset_threshold_to_target_is_ignored();
    test_stop_confirm();
    test_manual_mode();
    test_fault();
    test_fault_ack_resets_manual_screen();
    test_menu_settings_rejected_after_batch_started();
    test_menu_manual_rejected_after_batch_started();
    test_manual_screen_returns_to_main_when_not_in_manual();
    test_other_screens_ignore_long_press();
    test_diagnostics();
    test_menu_wraps();
    test_save_flow();
    test_format();
    test_labels();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    return 0;
}
