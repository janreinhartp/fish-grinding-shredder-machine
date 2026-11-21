#include "status/status_bus.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define COMMAND_QUEUE_DEPTH 8

static SemaphoreHandle_t s_mutex;
static QueueHandle_t s_queue;

static machine_status_t s_machine;
static io_snapshot_t s_io;
static scale_snapshot_t s_scale;
static machine_settings_t s_settings;
static bool s_start_allowed;
static bool s_settings_save_failed;
static bool s_settings_reset;

bool status_bus_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(operator_event_t));
    if (s_mutex == NULL || s_queue == NULL) {
        return false;
    }

    s_machine = (machine_status_t){
        .state = MACHINE_IDLE,
        .fault = FAULT_NONE,
        .outputs = {.shredder = false, .grinder = false, .door_open = false},
    };
    s_io = (io_snapshot_t){.link_ok = true, .attempted = false, .status = MODBUS_OK};
    s_scale = (scale_snapshot_t){.grams = 0,
                                 .last_ok = false,
                                 .have_sample = false,
                                 .sample_ms = 0,
                                 .status = MODBUS_OK,
                                 .status_known = false};
    s_settings = machine_settings_defaults();
    s_start_allowed = true;
    s_settings_save_failed = false;
    s_settings_reset = false;
    return true;
}

#define BUS_SET(var, value_ptr)                \
    do {                                       \
        xSemaphoreTake(s_mutex, portMAX_DELAY); \
        (var) = *(value_ptr);                  \
        xSemaphoreGive(s_mutex);               \
    } while (0)

#define BUS_GET(type, var)                      \
    do {                                        \
        type copy;                              \
        xSemaphoreTake(s_mutex, portMAX_DELAY); \
        copy = (var);                           \
        xSemaphoreGive(s_mutex);                \
        return copy;                            \
    } while (0)

void status_bus_set_machine(const machine_status_t *status) { BUS_SET(s_machine, status); }
machine_status_t status_bus_get_machine(void) { BUS_GET(machine_status_t, s_machine); }

void status_bus_set_io(const io_snapshot_t *io) { BUS_SET(s_io, io); }
io_snapshot_t status_bus_get_io(void) { BUS_GET(io_snapshot_t, s_io); }

void status_bus_set_scale(const scale_snapshot_t *scale) { BUS_SET(s_scale, scale); }
scale_snapshot_t status_bus_get_scale(void) { BUS_GET(scale_snapshot_t, s_scale); }

void status_bus_set_settings(const machine_settings_t *settings) { BUS_SET(s_settings, settings); }
machine_settings_t status_bus_get_settings(void) { BUS_GET(machine_settings_t, s_settings); }

void status_bus_set_start_allowed(bool allowed) { BUS_SET(s_start_allowed, &allowed); }
bool status_bus_get_start_allowed(void) { BUS_GET(bool, s_start_allowed); }

void status_bus_set_settings_save_failed(bool failed) { BUS_SET(s_settings_save_failed, &failed); }
bool status_bus_get_settings_save_failed(void) { BUS_GET(bool, s_settings_save_failed); }

void status_bus_set_settings_reset(bool reset) { BUS_SET(s_settings_reset, &reset); }
bool status_bus_get_settings_reset(void) { BUS_GET(bool, s_settings_reset); }

bool status_bus_send_command(const operator_event_t *event) {
    if (event == NULL || s_queue == NULL) {
        return false;
    }
    return xQueueSend(s_queue, event, 0) == pdTRUE;
}

bool status_bus_take_command(operator_event_t *event) {
    if (event == NULL || s_queue == NULL) {
        return false;
    }
    return xQueueReceive(s_queue, event, 0) == pdTRUE;
}
