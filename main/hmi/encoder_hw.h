#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint8_t a;     /* raw pin level */
    uint8_t b;     /* raw pin level */
    uint8_t sw;    /* raw pin level; 0 when pressed */
    uint32_t ms;   /* time of the edge, milliseconds since boot */
} encoder_sample_t;

/* Configures the encoder pins from device_profile() and installs the any-edge ISR. */
esp_err_t encoder_hw_init(void);

/* Non-blocking. Returns false when no sample is waiting. */
bool encoder_hw_take(encoder_sample_t *out);

/* The most recent pin levels, read under the port mux. */
encoder_sample_t encoder_hw_latest(void);
