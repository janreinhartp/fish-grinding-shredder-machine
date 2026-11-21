#include "device/profile.h"

/* Edit these values and rebuild to commission a different scale or wiring. */
static const device_profile_t k_profile = {
    .uart = {
        .port = 1,
        .tx_pin = 17,
        .rx_pin = 18,
        .de_pin = 16,
        .use_de = true,
        .baud = 9600,
        .parity = PROFILE_PARITY_NONE,
    },
    .scale = {
        .slave = 1,
        .function = 0x03,
        .reg = 0,
        .count = 2,
        .decode = SCALE_DECODE_S32_BE_GRAMS,
    },
    .io = {
        .slave = 2,
        .function = 0x0F,
        .coil = 0,
        .count = 3,
    },
    .panel = {
        .width = 320,
        .height = 240,
        .sclk_pin = 4,
        .mosi_pin = 5,
        .miso_pin = 6,
        .cs_pin = 7,
        .dc_pin = 8,
        .rst_pin = 9,
        .backlight_pin = 10,
        .backlight_active_high = true,
        .swap_xy = true,
        .mirror_x = false,
        .mirror_y = false,
        .spi_host = 2,
    },
    .encoder = {
        .a_pin = 11,
        .b_pin = 12,
        .switch_pin = 13,
        .switch_active_low = true,
    },
    .timing = {
        .poll_ms = 100,
        .refresh_ms = 500,
        .stale_ms = 500,
        .timeout_ms = 100,
        .control_period_ms = 10,
        .long_press_ms = 800,
        .button_debounce_ms = 20,
        .quadrature_debounce_ms = 2,
        .steps_per_detent = 4,
    },
};

const device_profile_t *device_profile(void) {
    return &k_profile;
}
