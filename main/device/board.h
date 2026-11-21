#pragma once
#include <stdbool.h>

#include "esp_err.h"
#include "modbus/modbus_master.h"

/* Installs the RS-485 UART from device_profile(). */
esp_err_t board_modbus_init(void);

/* Link whose write waits for TX done and flushes RX, and whose read ends on a 5 ms gap. */
const modbus_link_t *board_modbus_link(void);
