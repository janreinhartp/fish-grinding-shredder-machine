#include <stdbool.h>
#include <stdlib.h>

#include "device/board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hmi/display.h"
#include "hmi/encoder_hw.h"
#include "machine/machine.h"
#include "nvs_flash.h"
#include "settings/settings_store.h"
#include "status/status_bus.h"

static const char *TAG = "main";

#define TASK_STACK_BYTES 4096
#define HMI_TASK_STACK_BYTES 8192
#define HMI_TASK_PRIORITY 5
#define HMI_CPU 0
#define MODBUS_TASK_PRIORITY 9
#define CONTROL_TASK_PRIORITY 10
#define CONTROL_CPU 1

void hmi_task(void *arg);
void modbus_task(void *arg);
void control_task(void *arg);

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    machine_settings_t settings;
    bool loaded = settings_store_load(&settings);
    if (!loaded) {
        ESP_LOGW(TAG, "no valid stored settings; using defaults");
    }

    if (!status_bus_init()) {
        ESP_LOGE(TAG, "status bus allocation failed");
        abort();
    }
    status_bus_set_settings(&settings);
    status_bus_set_settings_reset(!loaded);

    ESP_ERROR_CHECK(board_modbus_init());
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(encoder_hw_init());

    if (xTaskCreatePinnedToCore(hmi_task, "hmi", HMI_TASK_STACK_BYTES, NULL, HMI_TASK_PRIORITY, NULL,
                                HMI_CPU) != pdPASS ||
        xTaskCreatePinnedToCore(modbus_task, "modbus", TASK_STACK_BYTES, NULL, MODBUS_TASK_PRIORITY, NULL,
                                CONTROL_CPU) != pdPASS ||
        xTaskCreatePinnedToCore(control_task, "control", TASK_STACK_BYTES, NULL, CONTROL_TASK_PRIORITY, NULL,
                                CONTROL_CPU) != pdPASS) {
        ESP_LOGE(TAG, "task creation failed");
        abort();
    }
}
