#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "device/scale_decode.h"

typedef enum {
    PROFILE_PARITY_NONE = 0,
    PROFILE_PARITY_EVEN,
    PROFILE_PARITY_ODD,
} profile_parity_t;

typedef struct {
    int port;
    int tx_pin;
    int rx_pin;
    int de_pin;
    bool use_de;
    uint32_t baud;
    profile_parity_t parity;
} profile_uart_t;

typedef struct {
    uint8_t slave;
    uint8_t function;
    uint16_t reg;
    uint16_t count;
    scale_decode_t decode;
} profile_scale_t;

typedef struct {
    uint8_t slave;
    uint8_t function;
    uint16_t coil;
    uint16_t count;
} profile_io_t;

typedef struct {
    int width;
    int height;
    int sclk_pin;
    int mosi_pin;
    int miso_pin;
    int cs_pin;
    int dc_pin;
    int rst_pin;
    int backlight_pin;
    bool backlight_active_high;
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
    int spi_host; /* 2 means SPI2; board.c maps it to SPI2_HOST */
} profile_panel_t;

typedef struct {
    int a_pin;
    int b_pin;
    int switch_pin;
    bool switch_active_low;
} profile_encoder_t;

typedef struct {
    uint32_t poll_ms;
    uint32_t refresh_ms;
    uint32_t stale_ms;
    uint32_t timeout_ms;
    uint32_t control_period_ms;
    uint32_t long_press_ms;
    uint32_t button_debounce_ms;
    uint32_t quadrature_debounce_ms;
    int steps_per_detent;
} profile_timing_t;

typedef struct {
    profile_uart_t uart;
    profile_scale_t scale;
    profile_io_t io;
    profile_panel_t panel;
    profile_encoder_t encoder;
    profile_timing_t timing;
} device_profile_t;

const device_profile_t *device_profile(void);
