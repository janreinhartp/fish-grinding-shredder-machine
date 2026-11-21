#include "test_harness.h"
#include "machine/machine.h"
#include "settings/settings_codec.h"

#include <string.h>

int g_failures = 0;

static void test_defaults_are_valid(void) {
    machine_settings_t s = machine_settings_defaults();
    EXPECT(machine_settings_valid(&s));
    EXPECT_EQ_INT(s.target_g, 25000);
    EXPECT_EQ_INT(s.minimum_batch_g, 1000);
    EXPECT_EQ_INT(s.tolerance_g, 100);
    EXPECT_EQ_INT(s.stable_ms, 2000);
    EXPECT_EQ_INT(s.reset_threshold_g, 500);
    EXPECT_EQ_INT(s.grinder_startup_ms, 2000);
    EXPECT_EQ_INT(s.shredder_startup_ms, 2000);
    EXPECT_EQ_INT(s.door_open_ms, 5000);
    EXPECT_EQ_INT(s.discharge_ms, 10000);
    EXPECT_EQ_INT(s.door_close_ms, 5000);
    EXPECT_EQ_INT(s.shredder_run_on_ms, 5000);
    EXPECT_EQ_INT(s.grinder_run_on_ms, 8000);
}

static void test_rejects_reset_not_below_target(void) {
    machine_settings_t s = machine_settings_defaults();
    s.reset_threshold_g = s.target_g;
    EXPECT(!machine_settings_valid(&s));
}

static void test_rejects_minimum_above_target(void) {
    machine_settings_t s = machine_settings_defaults();
    s.minimum_batch_g = s.target_g + 1;
    EXPECT(!machine_settings_valid(&s));
}

static void test_rejects_null_and_zero_tolerance(void) {
    EXPECT(!machine_settings_valid(NULL));
    machine_settings_t s = machine_settings_defaults();
    s.tolerance_g = 0;
    EXPECT(!machine_settings_valid(&s));
    s = machine_settings_defaults();
    s.stable_ms = 0;
    EXPECT(!machine_settings_valid(&s));
}

static void test_codec_round_trip(void) {
    machine_settings_t in = machine_settings_defaults();
    in.target_g = 18000;
    uint8_t buf[54];
    EXPECT_EQ_INT(settings_codec_size(), 54);
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    machine_settings_t out;
    memset(&out, 0, sizeof out);
    EXPECT(settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT_EQ_INT(out.target_g, 18000);
    EXPECT_EQ_INT(out.grinder_run_on_ms, 8000);
}

static void test_codec_rejects_bad_crc_and_version(void) {
    machine_settings_t in = machine_settings_defaults();
    uint8_t buf[54];
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    buf[10] ^= 0xFFu;
    machine_settings_t out = machine_settings_defaults();
    EXPECT(!settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT(settings_codec_pack(&in, buf, sizeof buf));
    buf[0] = 99;
    EXPECT(!settings_codec_unpack(buf, sizeof buf, &out));
    EXPECT(!settings_codec_unpack(buf, 10, &out));
    EXPECT(!settings_codec_pack(NULL, buf, sizeof buf));
}

int main(void) {
    test_defaults_are_valid();
    test_rejects_reset_not_below_target();
    test_rejects_minimum_above_target();
    test_rejects_null_and_zero_tolerance();
    test_codec_round_trip();
    test_codec_rejects_bad_crc_and_version();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
