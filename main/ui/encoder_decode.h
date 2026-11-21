#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENC_ROTATE = 0,
    ENC_SHORT,
    ENC_LONG,
} enc_event_type_t;

typedef struct {
    enc_event_type_t type;
    int8_t steps; /* +1 clockwise, -1 counterclockwise; 0 for button events */
} enc_event_t;

#define ENC_QUEUE_LEN 4

typedef struct {
    bool have_baseline;
    uint8_t ab;            /* last accepted quadrature state, (a << 1) | b */
    int8_t accum;          /* quadrature transitions since the last detent */
    uint32_t last_quad_ms; /* time of the last accepted quadrature edge */
    bool sw_down;          /* debounced switch state, true when pressed (low) */
    uint32_t last_sw_ms;   /* time of the last accepted switch edge */
    bool press_armed;      /* a press edge was seen and its release is pending */
    bool long_fired;
    uint32_t press_ms;
    enc_event_t queue[ENC_QUEUE_LEN];
    uint8_t q_head;
    uint8_t q_count;
} encoder_decode_t;

void encoder_decode_init(encoder_decode_t *dec);

/* Feed one sample of the encoder pins. The first sample is the baseline and
 * emits nothing. Returns true and fills *out when an event is available. At
 * most one event is returned per call; when a sample yields more than one,
 * the rest are returned by the following calls. Pass the same or a later
 * now_ms on those calls. now_ms wraps at 2^32. */
bool encoder_decode_step(encoder_decode_t *dec, uint32_t now_ms, uint8_t a, uint8_t b,
                         uint8_t sw, enc_event_t *out);
