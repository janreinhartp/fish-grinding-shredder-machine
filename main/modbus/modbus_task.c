#include <stdbool.h>
#include <stdint.h>

#include "device/board.h"
#include "device/profile.h"
#include "device/scale_decode.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus/modbus_master.h"
#include "status/status_bus.h"

static const char *TAG = "modbus";

#define SCALE_REGS_MAX 8
#define COIL_SHREDDER_BIT 0x01
#define COIL_GRINDER_BIT 0x02
#define COIL_DOOR_BIT 0x04

void modbus_task(void *arg);

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint8_t coil_byte(const output_image_t *outputs) {
    uint8_t value = 0;
    if (outputs->shredder) {
        value |= COIL_SHREDDER_BIT;
    }
    if (outputs->grinder) {
        value |= COIL_GRINDER_BIT;
    }
    if (outputs->door_open) {
        value |= COIL_DOOR_BIT;
    }
    return value;
}

static void poll_scale(const modbus_link_t *link, const profile_scale_t *cfg) {
    scale_snapshot_t snapshot = status_bus_get_scale();
    uint16_t regs[SCALE_REGS_MAX] = {0};
    int32_t grams = 0;

    modbus_status_t status = MODBUS_ERR_ARGUMENT;
    if (cfg->count > 0 && cfg->count <= SCALE_REGS_MAX) {
        status = modbus_read_registers(link, cfg->slave, cfg->function, cfg->reg, cfg->count, regs);
    }

    snapshot.status = status;
    snapshot.status_known = true;

    if (status == MODBUS_OK && scale_decode(cfg->decode, regs, cfg->count, &grams)) {
        snapshot.grams = grams;
        snapshot.last_ok = true;
        snapshot.have_sample = true;
        snapshot.sample_ms = now_ms();
    } else {
        snapshot.last_ok = false;
        ESP_LOGW(TAG, "scale read failed (modbus status %d)", (int)status);
    }
    status_bus_set_scale(&snapshot);
}

static modbus_status_t write_outputs(const modbus_link_t *link, const profile_io_t *cfg, uint8_t byte) {
    if (cfg->function == 0x0F) {
        return modbus_write_coils(link, cfg->slave, cfg->coil, cfg->count, byte);
    }

    modbus_status_t first_failure = MODBUS_OK;
    for (uint16_t i = 0; i < cfg->count && i < 8; i++) {
        bool on = (byte >> i) & 0x01;
        modbus_status_t status = modbus_write_single_coil(link, cfg->slave, (uint16_t)(cfg->coil + i), on);
        if (status != MODBUS_OK && first_failure == MODBUS_OK) {
            first_failure = status;
        }
    }
    return first_failure;
}

void modbus_task(void *arg) {
    (void)arg;
    const device_profile_t *profile = device_profile();
    const modbus_link_t *link = board_modbus_link();

    bool image_known = false; /* true once the slave has accepted last_byte */
    uint8_t last_byte = 0;
    uint32_t last_write_ms = 0;

    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(profile->timing.poll_ms);

    for (;;) {
        poll_scale(link, &profile->scale);

        machine_status_t machine = status_bus_get_machine();
        uint8_t wanted = coil_byte(&machine.outputs);
        uint32_t now = now_ms();
        bool refresh_due = (now - last_write_ms) >= profile->timing.refresh_ms;
        if (!image_known || wanted != last_byte || refresh_due) {
            modbus_status_t status = write_outputs(link, &profile->io, wanted);
            io_snapshot_t io = {.link_ok = (status == MODBUS_OK), .attempted = true, .status = status};
            status_bus_set_io(&io);
            if (status == MODBUS_OK) {
                image_known = true;
                last_byte = wanted;
                last_write_ms = now;
            } else {
                image_known = false; /* retry on the next poll */
                ESP_LOGW(TAG, "coil write failed (modbus status %d)", (int)status);
            }
        }

        vTaskDelayUntil(&wake, period > 0 ? period : 1);
    }
}
