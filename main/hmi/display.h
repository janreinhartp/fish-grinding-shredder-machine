#pragma once
#include "esp_err.h"

/* Brings up SPI, the ILI9341 panel, the backlight, and LVGL with one 320 by 240 display from
 * device_profile()->panel. The HMI task is the only caller of lv_timer_handler afterwards. */
esp_err_t display_init(void);
