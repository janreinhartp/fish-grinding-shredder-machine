#include "device/scale_decode.h"

bool scale_decode(scale_decode_t mode, const uint16_t *regs, size_t count, int32_t *grams) {
    if (regs == NULL || grams == NULL) {
        return false;
    }
    switch (mode) {
    case SCALE_DECODE_S32_BE_GRAMS:
        if (count != 2) {
            return false;
        }
        *grams = (int32_t)(((uint32_t)regs[0] << 16) | regs[1]);
        return true;
    case SCALE_DECODE_S32_LE_GRAMS:
        if (count != 2) {
            return false;
        }
        *grams = (int32_t)(((uint32_t)regs[1] << 16) | regs[0]);
        return true;
    case SCALE_DECODE_S16_GRAMS:
        if (count != 1) {
            return false;
        }
        *grams = (int16_t)regs[0];
        return true;
    default:
        return false;
    }
}
