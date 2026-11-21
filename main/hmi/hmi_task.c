#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "device/profile.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi/encoder_hw.h"
#include "lvgl.h"
#include "machine/machine.h"
#include "settings/settings_store.h"
#include "status/status_bus.h"
#include "ui/encoder_decode.h"
#include "ui/ui_logic.h"

static const char *TAG = "hmi";

#define HMI_PERIOD_MS 10
#define SCREEN_W 320
#define SCREEN_H 240
#define LIST_ROWS_MAX 11
#define SETTINGS_WINDOW 9
#define TEXT_LEN 96
#define WEIGHT_LEN 24

typedef struct {
    lv_obj_t *cont;
    lv_obj_t *title;
    lv_obj_t *rows[LIST_ROWS_MAX];
    int row_count;
} list_screen_t;

typedef struct {
    lv_obj_t *cont;
    lv_obj_t *weight;
    lv_obj_t *target;
    lv_obj_t *stable;
    lv_obj_t *state;
    lv_obj_t *shredder;
    lv_obj_t *grinder;
    lv_obj_t *door;
    lv_obj_t *scale_link;
    lv_obj_t *io_link;
    lv_obj_t *banner;
} main_screen_t;

static main_screen_t s_main;
static list_screen_t s_menu;
static list_screen_t s_stop;
static list_screen_t s_settings;
static list_screen_t s_manual_confirm;
static list_screen_t s_manual;
static list_screen_t s_diagnostics;
static list_screen_t s_fault;

void hmi_task(void *arg);

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ---------- widgets ---------- */

static lv_obj_t *make_container(void) {
    lv_obj_t *cont = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(cont);
    lv_obj_set_size(cont, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(cont, 0, 0);
    lv_obj_remove_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cont, LV_OBJ_FLAG_HIDDEN);
    return cont;
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, int x, int y, int width) {
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_pos(label, x, y);
    if (width > 0) {
        lv_obj_set_width(label, width);
    }
    lv_label_set_text(label, "");
    return label;
}

static void set_text(lv_obj_t *label, const char *text) {
    const char *current = lv_label_get_text(label);
    if (current == NULL || strcmp(current, text) != 0) {
        lv_label_set_text(label, text);
    }
}

static void make_main(main_screen_t *m) {
    m->cont = make_container();
    m->weight = make_label(m->cont, &lv_font_montserrat_20, 8, 8, 150);
    m->target = make_label(m->cont, &lv_font_montserrat_14, 8, 40, 150);
    m->stable = make_label(m->cont, &lv_font_montserrat_14, 8, 62, 150);
    m->state = make_label(m->cont, &lv_font_montserrat_14, 8, 84, 150);
    m->shredder = make_label(m->cont, &lv_font_montserrat_14, 170, 40, 146);
    m->grinder = make_label(m->cont, &lv_font_montserrat_14, 170, 62, 146);
    m->door = make_label(m->cont, &lv_font_montserrat_14, 170, 84, 146);
    m->scale_link = make_label(m->cont, &lv_font_montserrat_14, 8, 116, 150);
    m->io_link = make_label(m->cont, &lv_font_montserrat_14, 170, 116, 146);
    m->banner = make_label(m->cont, &lv_font_montserrat_14, 8, 148, 304);
    lv_obj_set_style_text_color(m->banner, lv_palette_main(LV_PALETTE_RED), 0);
}

static void make_list(list_screen_t *list, const char *title, int rows) {
    list->cont = make_container();
    list->title = make_label(list->cont, &lv_font_montserrat_20, 8, 6, 304);
    set_text(list->title, title);
    list->row_count = rows < LIST_ROWS_MAX ? rows : LIST_ROWS_MAX;
    for (int i = 0; i < list->row_count; i++) {
        list->rows[i] = make_label(list->cont, &lv_font_montserrat_14, 8, 36 + i * 19, 304);
    }
}

static void build_ui(void) {
    make_main(&s_main);
    make_list(&s_menu, "Menu", 3);
    make_list(&s_stop, "Stop batch?", 2);
    make_list(&s_settings, "Settings", SETTINGS_WINDOW);
    make_list(&s_manual_confirm, "Enable manual mode?", 2);
    make_list(&s_manual, "Manual", 4);
    make_list(&s_diagnostics, "Diagnostics", 10);
    make_list(&s_fault, "I/O link lost", 1);
    set_text(s_fault.rows[0], "Press to acknowledge");
}

static lv_obj_t *const *container_slots(void) {
    static lv_obj_t *slots[8];
    slots[0] = s_main.cont;
    slots[1] = s_menu.cont;
    slots[2] = s_stop.cont;
    slots[3] = s_settings.cont;
    slots[4] = s_manual_confirm.cont;
    slots[5] = s_manual.cont;
    slots[6] = s_diagnostics.cont;
    slots[7] = s_fault.cont;
    return slots;
}

/* Shows one container and hides the rest. */
static void show_only(lv_obj_t *shown) {
    lv_obj_t *const *slots = container_slots();
    for (int i = 0; i < 8; i++) {
        bool hidden = lv_obj_has_flag(slots[i], LV_OBJ_FLAG_HIDDEN);
        if (slots[i] == shown) {
            if (hidden) {
                lv_obj_remove_flag(slots[i], LV_OBJ_FLAG_HIDDEN);
            }
        } else if (!hidden) {
            lv_obj_add_flag(slots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

/* ---------- text ---------- */

static const char *on_off(bool on) {
    return on ? "On" : "Off";
}

static const char *decode_name(scale_decode_t decode) {
    switch (decode) {
    case SCALE_DECODE_S32_BE_GRAMS:
        return "S32_BE_GRAMS";
    case SCALE_DECODE_S32_LE_GRAMS:
        return "S32_LE_GRAMS";
    case SCALE_DECODE_S16_GRAMS:
        return "S16_GRAMS";
    }
    return "?";
}

static const char *modbus_status_name(modbus_status_t status) {
    switch (status) {
    case MODBUS_OK:
        return "OK";
    case MODBUS_ERR_TIMEOUT:
        return "TIMEOUT";
    case MODBUS_ERR_CRC:
        return "CRC";
    case MODBUS_ERR_SLAVE:
        return "SLAVE";
    case MODBUS_ERR_FUNCTION:
        return "FUNCTION";
    case MODBUS_ERR_EXCEPTION:
        return "EXCEPTION";
    case MODBUS_ERR_TRANSPORT:
        return "TRANSPORT";
    case MODBUS_ERR_LENGTH:
        return "LENGTH";
    case MODBUS_ERR_ARGUMENT:
        return "ARGUMENT";
    }
    return "?";
}

static const char *scale_link_text(const scale_snapshot_t *scale, uint32_t now) {
    if (!scale->have_sample) {
        return "Scale --";
    }
    if (scale->grams < 0) {
        return "Scale invalid";
    }
    if (!scale->last_ok) {
        return "Scale lost";
    }
    int32_t age = (int32_t)(now - scale->sample_ms);
    if (age > 0 && (uint32_t)age > device_profile()->timing.stale_ms) {
        return "Scale stale";
    }
    return "Scale OK";
}

static const char *io_link_text(const io_snapshot_t *io) {
    if (!io->attempted) {
        return "I/O --";
    }
    return io->link_ok ? "I/O OK" : "I/O lost";
}

static void setting_value_text(const machine_settings_t *s, int index, char *out, size_t len) {
    switch (index) {
    case 0:
        ui_format_weight(s->target_g, out, len);
        break;
    case 1:
        ui_format_weight(s->minimum_batch_g, out, len);
        break;
    case 2:
        ui_format_weight(s->tolerance_g, out, len);
        break;
    case 3:
        ui_format_time(s->stable_ms, out, len);
        break;
    case 4:
        ui_format_weight(s->reset_threshold_g, out, len);
        break;
    case 5:
        ui_format_time(s->grinder_startup_ms, out, len);
        break;
    case 6:
        ui_format_time(s->shredder_startup_ms, out, len);
        break;
    case 7:
        ui_format_time(s->door_open_ms, out, len);
        break;
    case 8:
        ui_format_time(s->discharge_ms, out, len);
        break;
    case 9:
        ui_format_time(s->door_close_ms, out, len);
        break;
    case 10:
        ui_format_time(s->shredder_run_on_ms, out, len);
        break;
    case 11:
        ui_format_time(s->grinder_run_on_ms, out, len);
        break;
    default:
        out[0] = '\0';
        break;
    }
}

/* ---------- rendering ---------- */

static void render_main(const machine_status_t *machine, const scale_snapshot_t *scale,
                        const io_snapshot_t *io, const machine_settings_t *published, uint32_t now) {
    char text[TEXT_LEN];
    char weight[WEIGHT_LEN];

    if (scale->have_sample) {
        ui_format_weight(scale->grams, weight, sizeof(weight));
        set_text(s_main.weight, weight);
    } else {
        set_text(s_main.weight, "--");
    }

    ui_format_weight(published->target_g, weight, sizeof(weight));
    snprintf(text, sizeof(text), "Target %s", weight);
    set_text(s_main.target, text);

    set_text(s_main.stable, machine->stable ? "Stable" : "Unstable");
    set_text(s_main.state, machine_state_name(machine->state));

    snprintf(text, sizeof(text), "Shredder %s", on_off(machine->outputs.shredder));
    set_text(s_main.shredder, text);
    snprintf(text, sizeof(text), "Grinder %s", on_off(machine->outputs.grinder));
    set_text(s_main.grinder, text);
    snprintf(text, sizeof(text), "Door command %s", machine->outputs.door_open ? "Open" : "Close");
    set_text(s_main.door, text);

    set_text(s_main.scale_link, scale_link_text(scale, now));
    set_text(s_main.io_link, io_link_text(io));

    char banner[TEXT_LEN * 3] = "";
    if (machine->scale_warning) {
        strcat(banner, "Scale lost during batch\n");
    }
    if (status_bus_get_settings_reset()) {
        strcat(banner, "Settings reset to defaults\n");
    }
    if (status_bus_get_settings_save_failed()) {
        strcat(banner, "Settings were not saved\n");
    }
    size_t len = strlen(banner);
    if (len > 0) {
        banner[len - 1] = '\0';
    }
    set_text(s_main.banner, banner);
}

static void render_row(list_screen_t *list, int index, bool selected, const char *text) {
    char line[TEXT_LEN + 4];
    snprintf(line, sizeof(line), "%s%s", selected ? "> " : "  ", text);
    set_text(list->rows[index], line);
}

static void render_menu(const ui_model_t *ui) {
    static const char *const rows[] = {"Settings", "Manual", "Diagnostics"};
    for (int i = 0; i < 3; i++) {
        render_row(&s_menu, i, ui->row == i, rows[i]);
    }
}

static void render_stop(const ui_model_t *ui) {
    static const char *const rows[] = {"Stop", "Cancel"};
    for (int i = 0; i < 2; i++) {
        render_row(&s_stop, i, ui->row == i, rows[i]);
    }
}

static void render_manual_confirm(const ui_model_t *ui) {
    static const char *const rows[] = {"Enable", "Cancel"};
    for (int i = 0; i < 2; i++) {
        render_row(&s_manual_confirm, i, ui->row == i, rows[i]);
    }
}

static void render_manual(const ui_model_t *ui, const machine_status_t *machine) {
    char text[TEXT_LEN];
    snprintf(text, sizeof(text), "Shredder %s", on_off(machine->outputs.shredder));
    render_row(&s_manual, 0, ui->row == 0, text);
    snprintf(text, sizeof(text), "Grinder %s", on_off(machine->outputs.grinder));
    render_row(&s_manual, 1, ui->row == 1, text);
    snprintf(text, sizeof(text), "Door command %s", machine->outputs.door_open ? "Open" : "Close");
    render_row(&s_manual, 2, ui->row == 2, text);
    render_row(&s_manual, 3, ui->row == 3, "Back");
}

static void render_settings(const ui_model_t *ui) {
    int first = ui->row - SETTINGS_WINDOW / 2;
    if (first > UI_SETTINGS_ROW_COUNT - SETTINGS_WINDOW) {
        first = UI_SETTINGS_ROW_COUNT - SETTINGS_WINDOW;
    }
    if (first < 0) {
        first = 0;
    }
    for (int i = 0; i < SETTINGS_WINDOW; i++) {
        int index = first + i;
        char value[WEIGHT_LEN];
        char text[TEXT_LEN];
        setting_value_text(&ui->settings, index, value, sizeof(value));
        bool selected = ui->row == index;
        const char *mark = (selected && ui->editing) ? "*" : "";
        snprintf(text, sizeof(text), "%s%s  %s", ui_setting_label(index), mark, value);
        render_row(&s_settings, i, selected, text);
    }
}

static void render_diagnostics(const machine_status_t *machine, const scale_snapshot_t *scale,
                               const io_snapshot_t *io, uint32_t now) {
    const device_profile_t *p = device_profile();
    char text[TEXT_LEN];
    char weight[WEIGHT_LEN];

    ui_format_weight(scale->grams, weight, sizeof(weight));
    snprintf(text, sizeof(text), "%s  %s", scale_link_text(scale, now), scale->have_sample ? weight : "--");
    set_text(s_diagnostics.rows[0], text);
    set_text(s_diagnostics.rows[1], io_link_text(io));
    snprintf(text, sizeof(text), "State %s", machine_state_name(machine->state));
    set_text(s_diagnostics.rows[2], text);
    snprintf(text, sizeof(text), "Fault %s", machine->fault == FAULT_IO_LINK ? "I/O link" : "None");
    set_text(s_diagnostics.rows[3], text);
    snprintf(text, sizeof(text), "Slave scale %u  I/O %u", (unsigned)p->scale.slave, (unsigned)p->io.slave);
    set_text(s_diagnostics.rows[4], text);
    snprintf(text, sizeof(text), "Baud %lu", (unsigned long)p->uart.baud);
    set_text(s_diagnostics.rows[5], text);
    snprintf(text, sizeof(text), "Scale register %u", (unsigned)p->scale.reg);
    set_text(s_diagnostics.rows[6], text);
    snprintf(text, sizeof(text), "Decode %s", decode_name(p->scale.decode));
    set_text(s_diagnostics.rows[7], text);
    snprintf(text, sizeof(text), "Last Modbus scale %s",
             scale->status_known ? modbus_status_name(scale->status) : "--");
    set_text(s_diagnostics.rows[8], text);
    snprintf(text, sizeof(text), "Last Modbus I/O %s", io->attempted ? modbus_status_name(io->status) : "--");
    set_text(s_diagnostics.rows[9], text);
}

static void render(const ui_model_t *ui, uint32_t now) {
    machine_status_t machine = status_bus_get_machine();
    scale_snapshot_t scale = status_bus_get_scale();
    io_snapshot_t io = status_bus_get_io();
    machine_settings_t published = status_bus_get_settings();

    if (machine.state == MACHINE_FAULT) {
        show_only(s_fault.cont);
        return;
    }

    switch (ui->screen) {
    case UI_SCREEN_MENU:
        render_menu(ui);
        show_only(s_menu.cont);
        break;
    case UI_SCREEN_STOP_CONFIRM:
        render_stop(ui);
        show_only(s_stop.cont);
        break;
    case UI_SCREEN_SETTINGS:
        render_settings(ui);
        show_only(s_settings.cont);
        break;
    case UI_SCREEN_MANUAL_CONFIRM:
        render_manual_confirm(ui);
        show_only(s_manual_confirm.cont);
        break;
    case UI_SCREEN_MANUAL:
        render_manual(ui, &machine);
        show_only(s_manual.cont);
        break;
    case UI_SCREEN_DIAGNOSTICS:
        render_diagnostics(&machine, &scale, &io, now);
        show_only(s_diagnostics.cont);
        break;
    case UI_SCREEN_MAIN:
    default:
        render_main(&machine, &scale, &io, &published, now);
        show_only(s_main.cont);
        break;
    }
}

/* ---------- input ---------- */

static void handle_event(ui_model_t *ui, const enc_event_t *event) {
    ui_view_t view = {
        .state = status_bus_get_machine().state,
        .settings = status_bus_get_settings(),
    };
    operator_event_t command = {.type = CMD_NONE, .level = false};
    ui_on_encoder(ui, &view, event, &command);
    if (command.type != CMD_NONE && !status_bus_send_command(&command)) {
        ESP_LOGW(TAG, "operator command %d dropped; queue full", (int)command.type);
    }
}

/* Feed one sample and drain every event it yields. Time never runs backwards. */
static void feed(encoder_decode_t *dec, ui_model_t *ui, uint32_t *last_ms, uint32_t t, uint8_t a, uint8_t b,
                 uint8_t sw) {
    if ((int32_t)(t - *last_ms) < 0) {
        t = *last_ms;
    }
    *last_ms = t;
    enc_event_t event;
    while (encoder_decode_step(dec, t, a, b, sw, &event)) {
        handle_event(ui, &event);
    }
}

static void service_save(ui_model_t *ui) {
    if (!ui->save_requested) {
        return;
    }
    bool ok = settings_store_save(&ui->settings);
    if (ok) {
        status_bus_set_settings(&ui->settings);
        status_bus_set_settings_reset(false);
        status_bus_set_settings_save_failed(false);
    } else {
        status_bus_set_settings_save_failed(true);
        ESP_LOGE(TAG, "settings save failed");
    }
    ui_on_settings_saved(ui, ok);
}

void hmi_task(void *arg) {
    (void)arg;

    ui_model_t ui;
    machine_settings_t published = status_bus_get_settings();
    ui_init(&ui, &published);

    encoder_decode_t decoder;
    encoder_decode_init(&decoder);
    uint32_t last_ms = now_ms();

    build_ui();

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(HMI_PERIOD_MS);

    for (;;) {
        encoder_sample_t sample;
        while (encoder_hw_take(&sample)) {
            feed(&decoder, &ui, &last_ms, sample.ms, sample.a, sample.b, sample.sw);
        }
        /* One more look at the latest levels so a held button can reach the long-press time. */
        encoder_sample_t latest = encoder_hw_latest();
        uint32_t now = now_ms();
        feed(&decoder, &ui, &last_ms, now, latest.a, latest.b, latest.sw);

        service_save(&ui);
        status_bus_set_start_allowed(ui_start_allowed(&ui));

        render(&ui, now);
        lv_timer_handler();

        vTaskDelayUntil(&wake, period > 0 ? period : 1);
    }
}
