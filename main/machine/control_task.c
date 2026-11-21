#include <stdbool.h>
#include <stdint.h>

#include "device/profile.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "machine/machine.h"
#include "status/status_bus.h"

static const char *TAG = "control";

static machine_core_t s_core;

void control_task(void *arg);

void control_task(void *arg) {
    (void)arg;
    const device_profile_t *profile = device_profile();

    machine_settings_t settings = status_bus_get_settings();
    if (!machine_core_init(&s_core, &settings)) {
        ESP_LOGE(TAG, "machine core rejected the stored settings; using defaults");
        settings = machine_settings_defaults();
        machine_core_init(&s_core, &settings);
    }

    machine_state_t last_state = machine_core_status(&s_core)->state;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(profile->timing.control_period_ms);

    for (;;) {
        vTaskDelayUntil(&wake, period > 0 ? period : 1);

        settings = status_bus_get_settings();
        machine_core_set_settings(&s_core, &settings);

        scale_snapshot_t scale = status_bus_get_scale();
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        int32_t age_ms = (int32_t)(now_ms - scale.sample_ms);
        if (age_ms < 0) {
            age_ms = 0; /* sample timestamp is slightly ahead of this clock read */
        }
        scale_sample_t sample = {
            .grams = scale.grams,
            .valid = scale.last_ok && scale.grams >= 0 && (uint32_t)age_ms <= profile->timing.stale_ms,
        };

        operator_event_t command = {.type = CMD_NONE, .level = false};
        const operator_event_t *command_ptr = NULL;
        if (status_bus_take_command(&command)) {
            command_ptr = &command;
        }

        bool start_allowed = status_bus_get_start_allowed();
        bool io_ok = status_bus_get_io().link_ok;

        machine_core_step(&s_core, now_ms, sample, io_ok, start_allowed, command_ptr);

        const machine_status_t *status = machine_core_status(&s_core);
        if (status->state != last_state) {
            ESP_LOGI(TAG, "%s -> %s", machine_state_name(last_state), machine_state_name(status->state));
            last_state = status->state;
        }
        status_bus_set_machine(status);
    }
}
