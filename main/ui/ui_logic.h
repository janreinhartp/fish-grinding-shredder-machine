#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "machine/machine.h"
#include "ui/encoder_decode.h"

typedef enum {
    UI_SCREEN_MAIN = 0,
    UI_SCREEN_MENU,
    UI_SCREEN_STOP_CONFIRM,
    UI_SCREEN_SETTINGS,
    UI_SCREEN_MANUAL_CONFIRM,
    UI_SCREEN_MANUAL,
    UI_SCREEN_DIAGNOSTICS
} ui_screen_t;

/* Settings rows: the twelve fields, then Save, then Back. */
#define UI_SETTING_COUNT 12
#define UI_SETTINGS_ROW_SAVE 12
#define UI_SETTINGS_ROW_BACK 13
#define UI_SETTINGS_ROW_COUNT 14

typedef struct {
    ui_screen_t screen;
    int row;       /* selected row on the current screen */
    bool editing;  /* settings: the selected field is armed for rotation */
    machine_settings_t settings; /* settings screen copy, edited in place */
    machine_settings_t saved;    /* last saved copy; restored by Back */
    bool save_requested;         /* set by Save until ui_on_settings_saved */
    bool manual_shredder;        /* manual screen: commanded levels */
    bool manual_grinder;
    bool manual_door;
} ui_model_t;

typedef struct {
    machine_state_t state;
    machine_settings_t settings;
} ui_view_t;

void ui_init(ui_model_t *ui, const machine_settings_t *settings);

/* Handle one encoder event. Always writes *command; it is CMD_NONE when the
 * event produces no machine command. */
void ui_on_encoder(ui_model_t *ui, const ui_view_t *view, const enc_event_t *event,
                   operator_event_t *command);

/* True only on the main screen. */
bool ui_start_allowed(const ui_model_t *ui);

/* Result of the HMI task's NVS write requested by Save. */
void ui_on_settings_saved(ui_model_t *ui, bool ok);

/* "25.00 kg", "-0.15 kg". */
void ui_format_weight(int32_t grams, char *out, size_t out_len);
/* "2.0 s". */
void ui_format_time(uint32_t ms, char *out, size_t out_len);

/* Row label for index 0..13; never NULL. */
const char *ui_setting_label(int index);
