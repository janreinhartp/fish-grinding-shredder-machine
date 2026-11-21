#include "ui/encoder_decode.h"

#include <string.h>

#define DETENT_TRANSITIONS 4
#define LONG_PRESS_MS 800u
#define SWITCH_DEBOUNCE_MS 20u
#define QUAD_DEBOUNCE_MS 2u

/* Indexed by (old_ab << 2) | new_ab. */
static const int8_t kQuadTable[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

static void push(encoder_decode_t *dec, enc_event_type_t type, int8_t steps) {
    if (dec->q_count >= ENC_QUEUE_LEN) {
        return;
    }
    uint8_t tail = (uint8_t)((dec->q_head + dec->q_count) % ENC_QUEUE_LEN);
    dec->queue[tail].type = type;
    dec->queue[tail].steps = steps;
    dec->q_count++;
}

void encoder_decode_init(encoder_decode_t *dec) {
    memset(dec, 0, sizeof(*dec));
}

bool encoder_decode_step(encoder_decode_t *dec, uint32_t now_ms, uint8_t a, uint8_t b,
                         uint8_t sw, enc_event_t *out) {
    uint8_t ab = (uint8_t)(((a ? 1u : 0u) << 1) | (b ? 1u : 0u));
    bool down = (sw == 0);

    if (!dec->have_baseline) {
        dec->have_baseline = true;
        dec->ab = ab;
        dec->sw_down = down;
        dec->last_quad_ms = now_ms - QUAD_DEBOUNCE_MS;
    } else {
        if (down == dec->sw_down) {
            dec->sw_pending = false;
        } else if (!dec->sw_pending || dec->sw_pending_down != down) {
            dec->sw_pending = true;
            dec->sw_pending_down = down;
            dec->sw_pending_ms = now_ms;
        } else if ((uint32_t)(now_ms - dec->sw_pending_ms) >= SWITCH_DEBOUNCE_MS) {
            dec->sw_down = down;
            dec->sw_pending = false;
            if (down) {
                dec->press_armed = true;
                dec->long_fired = false;
                dec->press_ms = now_ms;
            } else {
                if (dec->press_armed && !dec->long_fired) {
                    push(dec, ENC_SHORT, 0);
                }
                dec->press_armed = false;
            }
        }

        /* A hold that reached the threshold fires before a later release is accepted. */
        if (dec->press_armed && dec->sw_down && !dec->long_fired &&
            (uint32_t)(now_ms - dec->press_ms) >= LONG_PRESS_MS) {
            dec->long_fired = true;
            push(dec, ENC_LONG, 0);
        }

        if (ab != dec->ab && (uint32_t)(now_ms - dec->last_quad_ms) >= QUAD_DEBOUNCE_MS) {
            dec->accum = (int8_t)(dec->accum + kQuadTable[(dec->ab << 2) | ab]);
            dec->ab = ab;
            dec->last_quad_ms = now_ms;
            if (dec->accum >= DETENT_TRANSITIONS) {
                dec->accum = 0;
                push(dec, ENC_ROTATE, +1);
            } else if (dec->accum <= -DETENT_TRANSITIONS) {
                dec->accum = 0;
                push(dec, ENC_ROTATE, -1);
            }
        }
    }

    if (dec->q_count == 0) {
        return false;
    }
    *out = dec->queue[dec->q_head];
    dec->q_head = (uint8_t)((dec->q_head + 1) % ENC_QUEUE_LEN);
    dec->q_count--;
    return true;
}
