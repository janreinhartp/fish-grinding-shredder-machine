#include "ui/ui_logic.h"

#include <stddef.h>
#include <stdio.h>

enum { MENU_SETTINGS = 0, MENU_MANUAL, MENU_DIAGNOSTICS, MENU_ROWS };
enum { STOP_STOP = 0, STOP_CANCEL, STOP_ROWS };
enum { CONFIRM_ENABLE = 0, CONFIRM_CANCEL, CONFIRM_ROWS };
enum { MANUAL_SHREDDER = 0, MANUAL_GRINDER, MANUAL_DOOR, MANUAL_BACK, MANUAL_ROWS };

typedef struct {
    const char *label;
    size_t offset;
    bool is_signed;
    int64_t min;
    int64_t max;
    int64_t step;
} setting_field_t;

#define FIELD_S(name, lo, hi, st) \
    {#name, offsetof(machine_settings_t, name), true, (lo), (hi), (st)}
#define FIELD_U(name, lo, hi, st) \
    {#name, offsetof(machine_settings_t, name), false, (lo), (hi), (st)}

static const setting_field_t FIELDS[UI_SETTING_COUNT] = {
    FIELD_S(target_g, 100, 50000, 100),
    FIELD_S(minimum_batch_g, 100, 50000, 100),
    FIELD_S(tolerance_g, 10, 2000, 10),
    FIELD_U(stable_ms, 500, 30000, 100),
    FIELD_S(reset_threshold_g, 0, 50000, 100),
    FIELD_U(grinder_startup_ms, 0, 120000, 100),
    FIELD_U(shredder_startup_ms, 0, 120000, 100),
    FIELD_U(door_open_ms, 0, 120000, 100),
    FIELD_U(discharge_ms, 0, 300000, 100),
    FIELD_U(door_close_ms, 0, 120000, 100),
    FIELD_U(shredder_run_on_ms, 0, 300000, 100),
    FIELD_U(grinder_run_on_ms, 0, 300000, 100),
};

static int64_t field_get(const machine_settings_t *s, const setting_field_t *f) {
    const unsigned char *base = (const unsigned char *)s + f->offset;
    if (f->is_signed) {
        return *(const int32_t *)(const void *)base;
    }
    return *(const uint32_t *)(const void *)base;
}

static void field_set(machine_settings_t *s, const setting_field_t *f, int64_t v) {
    unsigned char *base = (unsigned char *)s + f->offset;
    if (f->is_signed) {
        *(int32_t *)(void *)base = (int32_t)v;
    } else {
        *(uint32_t *)(void *)base = (uint32_t)v;
    }
}

/* Move the selection by steps, wrapping in both directions. */
static int wrap_row(int row, int steps, int count) {
    int r = (row + steps) % count;
    return r < 0 ? r + count : r;
}

static int row_count(const ui_model_t *ui) {
    switch (ui->screen) {
    case UI_SCREEN_MENU:
        return MENU_ROWS;
    case UI_SCREEN_STOP_CONFIRM:
        return STOP_ROWS;
    case UI_SCREEN_SETTINGS:
        return UI_SETTINGS_ROW_COUNT;
    case UI_SCREEN_MANUAL_CONFIRM:
        return CONFIRM_ROWS;
    case UI_SCREEN_MANUAL:
        return MANUAL_ROWS;
    default:
        return 0;
    }
}

static void go(ui_model_t *ui, ui_screen_t screen, int row) {
    ui->screen = screen;
    ui->row = row;
    ui->editing = false;
}

static void edit_selected(ui_model_t *ui, int steps) {
    const setting_field_t *f = &FIELDS[ui->row];
    int count = steps < 0 ? -steps : steps;
    int64_t dir = steps < 0 ? -1 : 1;
    for (int i = 0; i < count; i++) {
        machine_settings_t candidate = ui->settings;
        int64_t v = field_get(&candidate, f) + dir * f->step;
        if (v < f->min || v > f->max) {
            continue;
        }
        field_set(&candidate, f, v);
        if (!machine_settings_valid(&candidate)) {
            continue;
        }
        ui->settings = candidate;
    }
}

static void on_short_press(ui_model_t *ui, operator_event_t *command) {
    switch (ui->screen) {
    case UI_SCREEN_MENU:
        if (ui->row == MENU_SETTINGS) {
            ui->settings = ui->saved;
            go(ui, UI_SCREEN_SETTINGS, 0);
        } else if (ui->row == MENU_MANUAL) {
            go(ui, UI_SCREEN_MANUAL_CONFIRM, CONFIRM_ENABLE);
        } else {
            go(ui, UI_SCREEN_DIAGNOSTICS, 0);
        }
        break;
    case UI_SCREEN_STOP_CONFIRM:
        if (ui->row == STOP_STOP) {
            command->type = CMD_STOP_CONFIRMED;
        }
        go(ui, UI_SCREEN_MAIN, 0);
        break;
    case UI_SCREEN_MANUAL_CONFIRM:
        if (ui->row == CONFIRM_ENABLE) {
            command->type = CMD_ENTER_MANUAL;
            ui->manual_shredder = false;
            ui->manual_grinder = false;
            ui->manual_door = false;
            go(ui, UI_SCREEN_MANUAL, MANUAL_SHREDDER);
        } else {
            go(ui, UI_SCREEN_MENU, MENU_MANUAL);
        }
        break;
    case UI_SCREEN_MANUAL:
        if (ui->row == MANUAL_SHREDDER) {
            ui->manual_shredder = !ui->manual_shredder;
            command->type = CMD_SET_SHREDDER;
            command->level = ui->manual_shredder;
        } else if (ui->row == MANUAL_GRINDER) {
            ui->manual_grinder = !ui->manual_grinder;
            command->type = CMD_SET_GRINDER;
            command->level = ui->manual_grinder;
        } else if (ui->row == MANUAL_DOOR) {
            ui->manual_door = !ui->manual_door;
            command->type = CMD_SET_DOOR;
            command->level = ui->manual_door;
        } else {
            command->type = CMD_EXIT_MANUAL;
            go(ui, UI_SCREEN_MAIN, 0);
        }
        break;
    case UI_SCREEN_SETTINGS:
        if (ui->row < UI_SETTING_COUNT) {
            ui->editing = !ui->editing;
        } else if (ui->row == UI_SETTINGS_ROW_SAVE) {
            ui->save_requested = true;
        } else {
            ui->settings = ui->saved;
            go(ui, UI_SCREEN_MENU, MENU_SETTINGS);
        }
        break;
    case UI_SCREEN_DIAGNOSTICS:
        go(ui, UI_SCREEN_MENU, MENU_DIAGNOSTICS);
        break;
    case UI_SCREEN_MAIN:
    default:
        break;
    }
}

static void on_long_press(ui_model_t *ui, machine_state_t state) {
    if (ui->screen != UI_SCREEN_MAIN) {
        return;
    }
    if (state == MACHINE_IDLE) {
        go(ui, UI_SCREEN_MENU, MENU_SETTINGS);
    } else if (state != MACHINE_FAULT && state != MACHINE_MANUAL) {
        go(ui, UI_SCREEN_STOP_CONFIRM, STOP_STOP);
    }
}

void ui_init(ui_model_t *ui, const machine_settings_t *settings) {
    ui->screen = UI_SCREEN_MAIN;
    ui->row = 0;
    ui->editing = false;
    ui->settings = *settings;
    ui->saved = *settings;
    ui->save_requested = false;
    ui->manual_shredder = false;
    ui->manual_grinder = false;
    ui->manual_door = false;
}

void ui_on_encoder(ui_model_t *ui, const ui_view_t *view, const enc_event_t *event,
                   operator_event_t *command) {
    command->type = CMD_NONE;
    command->level = false;

    if (view->state == MACHINE_FAULT) {
        if (event->type == ENC_SHORT) {
            command->type = CMD_ACK_FAULT;
            ui->manual_shredder = false;
            ui->manual_grinder = false;
            ui->manual_door = false;
            go(ui, UI_SCREEN_MAIN, 0);
        }
        return;
    }

    switch (event->type) {
    case ENC_ROTATE:
        if (ui->screen == UI_SCREEN_SETTINGS && ui->editing) {
            edit_selected(ui, event->steps);
        } else if (row_count(ui) > 0) {
            ui->row = wrap_row(ui->row, event->steps, row_count(ui));
        }
        break;
    case ENC_SHORT:
        on_short_press(ui, command);
        break;
    case ENC_LONG:
        on_long_press(ui, view->state);
        break;
    }
}

bool ui_start_allowed(const ui_model_t *ui) {
    return ui->screen == UI_SCREEN_MAIN;
}

void ui_on_settings_saved(ui_model_t *ui, bool ok) {
    ui->save_requested = false;
    if (ok) {
        ui->saved = ui->settings;
        if (ui->screen == UI_SCREEN_SETTINGS) {
            go(ui, UI_SCREEN_MENU, MENU_SETTINGS);
        }
    }
}

void ui_format_weight(int32_t grams, char *out, size_t out_len) {
    if (out == NULL || out_len == 0) {
        return;
    }
    int64_t mag = grams < 0 ? -(int64_t)grams : (int64_t)grams;
    int64_t hundredths = (mag + 5) / 10;
    const char *sign = (grams < 0 && hundredths != 0) ? "-" : "";
    snprintf(out, out_len, "%s%lld.%02lld kg", sign, (long long)(hundredths / 100),
             (long long)(hundredths % 100));
}

void ui_format_time(uint32_t ms, char *out, size_t out_len) {
    if (out == NULL || out_len == 0) {
        return;
    }
    uint64_t tenths = ((uint64_t)ms + 50u) / 100u;
    snprintf(out, out_len, "%llu.%llu s", (unsigned long long)(tenths / 10),
             (unsigned long long)(tenths % 10));
}

const char *ui_setting_label(int index) {
    if (index >= 0 && index < UI_SETTING_COUNT) {
        return FIELDS[index].label;
    }
    if (index == UI_SETTINGS_ROW_SAVE) {
        return "Save";
    }
    if (index == UI_SETTINGS_ROW_BACK) {
        return "Back";
    }
    return "";
}
