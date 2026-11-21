#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    SCALE_DECODE_S32_BE_GRAMS = 0,
    SCALE_DECODE_S32_LE_GRAMS,
    SCALE_DECODE_S16_GRAMS,
} scale_decode_t;

/* Decode the scale registers into signed grams. Returns false and leaves
 * *grams untouched for a null pointer, an unknown mode, or a wrong count. */
bool scale_decode(scale_decode_t mode, const uint16_t *regs, size_t count, int32_t *grams);
