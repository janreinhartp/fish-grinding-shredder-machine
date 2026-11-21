#include "test_harness.h"
#include "ui/encoder_decode.h"

int g_failures = 0;

#define MAX_EVENTS 4

/* Feed one sample and drain every event it produces. Returns the event count. */
static int feed(encoder_decode_t *dec, uint32_t t, uint8_t ab, uint8_t sw, enc_event_t *ev) {
    int n = 0;
    enc_event_t e;
    while (n < MAX_EVENTS && encoder_decode_step(dec, t, (ab >> 1) & 1u, ab & 1u, sw, &e)) {
        ev[n++] = e;
    }
    return n;
}

static void test_first_sample_is_baseline(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0, 0x2, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 5, 0x2, 0, ev), 0);
    /* Boot with the switch held: the eventual release is not a press. */
    EXPECT_EQ_INT(feed(&dec, 100, 0x2, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1000, 0x2, 1, ev), 0);
}

static void test_clockwise_detent(void) {
    static const uint8_t seq[] = {0x0, 0x1, 0x3, 0x2, 0x0};
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    for (int i = 0; i < 5; i++) {
        int n = feed(&dec, (uint32_t)(i * 5), seq[i], 1, ev);
        if (i < 4) {
            EXPECT_EQ_INT(n, 0);
        } else {
            EXPECT_EQ_INT(n, 1);
            EXPECT_EQ_INT(ev[0].type, ENC_ROTATE);
            EXPECT_EQ_INT(ev[0].steps, 1);
        }
    }
}

static void test_counterclockwise_detent(void) {
    static const uint8_t seq[] = {0x0, 0x2, 0x3, 0x1, 0x0};
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    for (int i = 0; i < 5; i++) {
        int n = feed(&dec, (uint32_t)(i * 5), seq[i], 1, ev);
        if (i < 4) {
            EXPECT_EQ_INT(n, 0);
        } else {
            EXPECT_EQ_INT(n, 1);
            EXPECT_EQ_INT(ev[0].type, ENC_ROTATE);
            EXPECT_EQ_INT(ev[0].steps, -1);
        }
    }
}

static void test_two_detents_emit_two_events(void) {
    static const uint8_t seq[] = {0x0, 0x1, 0x3, 0x2, 0x0, 0x1, 0x3, 0x2, 0x0};
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    for (int i = 0; i < 9; i++) {
        total += feed(&dec, (uint32_t)(i * 5), seq[i], 1, ev);
    }
    EXPECT_EQ_INT(total, 2);
}

static void test_quadrature_edge_under_2ms_is_ignored(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    total += feed(&dec, 0, 0x0, 1, ev);
    total += feed(&dec, 5, 0x1, 1, ev);   /* +1 */
    total += feed(&dec, 6, 0x3, 1, ev);   /* 1 ms later: ignored */
    total += feed(&dec, 7, 0x2, 1, ev);   /* 01 -> 10 is invalid, counts 0 */
    total += feed(&dec, 12, 0x0, 1, ev);  /* +1: total 2 of 4, no detent */
    EXPECT_EQ_INT(total, 0);
}

static void test_quadrature_edge_at_2ms_is_accepted(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    total += feed(&dec, 0, 0x0, 1, ev);
    total += feed(&dec, 5, 0x1, 1, ev);
    total += feed(&dec, 7, 0x3, 1, ev);
    total += feed(&dec, 9, 0x2, 1, ev);
    EXPECT_EQ_INT(total, 0);
    EXPECT_EQ_INT(feed(&dec, 11, 0x0, 1, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_ROTATE);
    EXPECT_EQ_INT(ev[0].steps, 1);
}

static void test_short_press(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1000, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1020, 0x0, 0, ev), 0); /* level stable: press accepted */
    EXPECT_EQ_INT(feed(&dec, 1100, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1119, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1120, 0x0, 1, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_SHORT);
}

static void test_long_press_fires_once_and_release_is_silent(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 100, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 120, 0x0, 0, ev), 0); /* press accepted */
    EXPECT_EQ_INT(feed(&dec, 919, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 920, 0x0, 0, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_LONG);
    EXPECT_EQ_INT(feed(&dec, 1500, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 1520, 0x0, 1, ev), 0);
}

static void test_release_at_long_threshold_emits_long_not_short(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 100, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 120, 0x0, 0, ev), 0); /* press accepted */
    /* Release sampled as the hold reaches 800 ms: long, not short. */
    EXPECT_EQ_INT(feed(&dec, 920, 0x0, 1, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_LONG);
    EXPECT_EQ_INT(feed(&dec, 940, 0x0, 1, ev), 0);
}

static void test_10ms_pulse_emits_nothing(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    total += feed(&dec, 0, 0x0, 1, ev);
    total += feed(&dec, 100, 0x0, 0, ev);
    total += feed(&dec, 110, 0x0, 1, ev);
    total += feed(&dec, 200, 0x0, 1, ev);
    EXPECT_EQ_INT(total, 0);
}

static void test_1ms_spike_emits_nothing(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    total += feed(&dec, 0, 0x0, 1, ev);
    total += feed(&dec, 100, 0x0, 0, ev);
    total += feed(&dec, 101, 0x0, 1, ev);
    total += feed(&dec, 200, 0x0, 1, ev);
    EXPECT_EQ_INT(total, 0);
}

static void test_press_bounce_under_20ms_is_ignored(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    int total = 0;
    encoder_decode_init(&dec);
    total += feed(&dec, 0, 0x0, 1, ev);
    total += feed(&dec, 100, 0x0, 0, ev);
    total += feed(&dec, 105, 0x0, 1, ev); /* bounce up before 20 ms */
    total += feed(&dec, 108, 0x0, 0, ev); /* down again */
    total += feed(&dec, 128, 0x0, 0, ev); /* this press is accepted */
    EXPECT_EQ_INT(total, 0);
    EXPECT_EQ_INT(feed(&dec, 200, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 220, 0x0, 1, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_SHORT);
}

static void test_two_events_from_one_sample_arrive_on_successive_calls(void) {
    static const uint8_t seq[] = {0x1, 0x3, 0x2};
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    enc_event_t e;
    int rotates = 0, longs = 0;
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 10, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 30, 0x0, 0, ev), 0); /* press accepted */
    EXPECT_EQ_INT(feed(&dec, 100, seq[0], 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 105, seq[1], 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 110, seq[2], 0, ev), 0);
    /* This sample both completes the detent and crosses the 800 ms hold. */
    EXPECT(encoder_decode_step(&dec, 830, 0, 0, 0, &e));
    if (e.type == ENC_ROTATE) { rotates++; EXPECT_EQ_INT(e.steps, 1); }
    if (e.type == ENC_LONG) { longs++; }
    EXPECT(encoder_decode_step(&dec, 830, 0, 0, 0, &e));
    if (e.type == ENC_ROTATE) { rotates++; EXPECT_EQ_INT(e.steps, 1); }
    if (e.type == ENC_LONG) { longs++; }
    EXPECT_EQ_INT(rotates, 1);
    EXPECT_EQ_INT(longs, 1);
    EXPECT(!encoder_decode_step(&dec, 830, 0, 0, 0, &e));
}

static void test_timestamp_wraparound(void) {
    encoder_decode_t dec;
    enc_event_t ev[MAX_EVENTS];
    encoder_decode_init(&dec);
    EXPECT_EQ_INT(feed(&dec, 0xFFFFFF00u, 0x0, 1, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 0xFFFFFF10u, 0x0, 0, ev), 0);
    EXPECT_EQ_INT(feed(&dec, 0xFFFFFF10u + 20u, 0x0, 0, ev), 0); /* press accepted across the clock */
    EXPECT_EQ_INT(feed(&dec, 0xFFFFFF10u + 20u + 800u, 0x0, 0, ev), 1);
    EXPECT_EQ_INT(ev[0].type, ENC_LONG);
}

int main(void) {
    test_first_sample_is_baseline();
    test_clockwise_detent();
    test_counterclockwise_detent();
    test_two_detents_emit_two_events();
    test_quadrature_edge_under_2ms_is_ignored();
    test_quadrature_edge_at_2ms_is_accepted();
    test_short_press();
    test_long_press_fires_once_and_release_is_silent();
    test_release_at_long_threshold_emits_long_not_short();
    test_10ms_pulse_emits_nothing();
    test_1ms_spike_emits_nothing();
    test_press_bounce_under_20ms_is_ignored();
    test_two_events_from_one_sample_arrive_on_successive_calls();
    test_timestamp_wraparound();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
