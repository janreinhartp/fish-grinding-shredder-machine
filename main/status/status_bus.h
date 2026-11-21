#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "machine/machine.h"

typedef struct {
    bool link_ok;
    bool attempted;
} io_snapshot_t;

typedef struct {
    int32_t grams;
    bool last_ok;
    bool have_sample;
    uint32_t sample_ms;
} scale_snapshot_t;

/* Creates the mutex and the 8-deep operator event queue. Returns false on failure. */
bool status_bus_init(void);

void status_bus_set_machine(const machine_status_t *status);
machine_status_t status_bus_get_machine(void);

void status_bus_set_io(const io_snapshot_t *io);
io_snapshot_t status_bus_get_io(void);

void status_bus_set_scale(const scale_snapshot_t *scale);
scale_snapshot_t status_bus_get_scale(void);

void status_bus_set_settings(const machine_settings_t *settings);
machine_settings_t status_bus_get_settings(void);

void status_bus_set_start_allowed(bool allowed);
bool status_bus_get_start_allowed(void);

void status_bus_set_settings_save_pending(bool pending);
bool status_bus_get_settings_save_pending(void);

void status_bus_set_settings_reset(bool reset);
bool status_bus_get_settings_reset(void);

/* Non-blocking. send returns false when the queue is full; take returns false when empty. */
bool status_bus_send_command(const operator_event_t *event);
bool status_bus_take_command(operator_event_t *event);
