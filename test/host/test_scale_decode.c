#include "test_harness.h"
#include "device/scale_decode.h"
#include "device/profile.h"

int g_failures = 0;

static void test_s32_be(void) {
    const uint16_t regs[] = {0x0000, 0x6720};
    int32_t g = 0;
    EXPECT(scale_decode(SCALE_DECODE_S32_BE_GRAMS, regs, 2, &g));
    EXPECT_EQ_INT(g, 26400);
}

static void test_s32_le(void) {
    const uint16_t regs[] = {0x6720, 0x0000};
    int32_t g = 0;
    EXPECT(scale_decode(SCALE_DECODE_S32_LE_GRAMS, regs, 2, &g));
    EXPECT_EQ_INT(g, 26400);
}

static void test_s32_negative(void) {
    const uint16_t regs[] = {0xFFFF, 0xFFFF};
    int32_t g = 0;
    EXPECT(scale_decode(SCALE_DECODE_S32_BE_GRAMS, regs, 2, &g));
    EXPECT_EQ_INT(g, -1);
}

static void test_s16(void) {
    const uint16_t regs[] = {0xFF9C, 0x0000};
    int32_t g = 0;
    EXPECT(scale_decode(SCALE_DECODE_S16_GRAMS, regs, 1, &g));
    EXPECT_EQ_INT(g, -100);
}

static void test_s16_wrong_count(void) {
    const uint16_t regs[] = {0xFF9C, 0x0000};
    int32_t g = 7;
    EXPECT(!scale_decode(SCALE_DECODE_S16_GRAMS, regs, 2, &g));
    EXPECT_EQ_INT(g, 7);
}

static void test_s32_wrong_count(void) {
    const uint16_t regs[] = {0x0000, 0x6720};
    int32_t g = 7;
    EXPECT(!scale_decode(SCALE_DECODE_S32_BE_GRAMS, regs, 1, &g));
    EXPECT(!scale_decode(SCALE_DECODE_S32_LE_GRAMS, regs, 3, &g));
    EXPECT_EQ_INT(g, 7);
}

static void test_bad_mode_and_null(void) {
    const uint16_t regs[] = {0x0000, 0x6720};
    int32_t g = 7;
    EXPECT(!scale_decode((scale_decode_t)99, regs, 2, &g));
    EXPECT(!scale_decode(SCALE_DECODE_S32_BE_GRAMS, NULL, 2, &g));
    EXPECT(!scale_decode(SCALE_DECODE_S32_BE_GRAMS, regs, 2, NULL));
    EXPECT_EQ_INT(g, 7);
}

static void test_profile(void) {
    const device_profile_t *p = device_profile();
    EXPECT(p != NULL);
    if (p == NULL) {
        return;
    }
    EXPECT_EQ_INT(p->uart.baud, 9600);
    EXPECT_EQ_INT(p->uart.tx_pin, 17);
    EXPECT_EQ_INT(p->uart.rx_pin, 18);
    EXPECT_EQ_INT(p->uart.de_pin, 16);
    EXPECT(p->uart.use_de);
    EXPECT_EQ_INT(p->uart.parity, PROFILE_PARITY_NONE);
    EXPECT_EQ_INT(p->uart.port, 1);

    EXPECT_EQ_INT(p->scale.slave, 1);
    EXPECT_EQ_INT(p->scale.function, 0x03);
    EXPECT_EQ_INT(p->scale.reg, 0);
    EXPECT_EQ_INT(p->scale.count, 2);
    EXPECT_EQ_INT(p->scale.decode, SCALE_DECODE_S32_BE_GRAMS);

    EXPECT_EQ_INT(p->io.slave, 2);
    EXPECT_EQ_INT(p->io.function, 0x0F);
    EXPECT_EQ_INT(p->io.coil, 0);
    EXPECT_EQ_INT(p->io.count, 3);

    EXPECT_EQ_INT(p->panel.width, 320);
    EXPECT_EQ_INT(p->panel.height, 240);
    EXPECT_EQ_INT(p->panel.sclk_pin, 4);
    EXPECT_EQ_INT(p->panel.mosi_pin, 5);
    EXPECT_EQ_INT(p->panel.miso_pin, 6);
    EXPECT_EQ_INT(p->panel.cs_pin, 7);
    EXPECT_EQ_INT(p->panel.dc_pin, 8);
    EXPECT_EQ_INT(p->panel.rst_pin, 9);
    EXPECT_EQ_INT(p->panel.backlight_pin, 10);
    EXPECT(p->panel.backlight_active_high);
    EXPECT(p->panel.swap_xy);
    EXPECT(!p->panel.mirror_x);
    EXPECT(!p->panel.mirror_y);
    EXPECT_EQ_INT(p->panel.spi_host, 2);

    EXPECT_EQ_INT(p->encoder.a_pin, 11);
    EXPECT_EQ_INT(p->encoder.b_pin, 12);
    EXPECT_EQ_INT(p->encoder.switch_pin, 13);
    EXPECT(p->encoder.switch_active_low);

    EXPECT_EQ_INT(p->timing.poll_ms, 100);
    EXPECT_EQ_INT(p->timing.refresh_ms, 500);
    EXPECT_EQ_INT(p->timing.stale_ms, 500);
    EXPECT_EQ_INT(p->timing.timeout_ms, 100);
    EXPECT_EQ_INT(p->timing.control_period_ms, 10);
    EXPECT_EQ_INT(p->timing.long_press_ms, 800);
    EXPECT_EQ_INT(p->timing.button_debounce_ms, 20);
    EXPECT_EQ_INT(p->timing.quadrature_debounce_ms, 2);
    EXPECT_EQ_INT(p->timing.steps_per_detent, 4);
}

int main(void) {
    test_s32_be();
    test_s32_le();
    test_s32_negative();
    test_s16();
    test_s16_wrong_count();
    test_s32_wrong_count();
    test_bad_mode_and_null();
    test_profile();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
