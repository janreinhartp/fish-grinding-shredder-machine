#include "test_harness.h"
#include "common/crc16.h"

int g_failures = 0;

static void test_ascii_vector(void) {
    const uint8_t data[] = "123456789";
    EXPECT_EQ_INT(crc16_modbus(data, 9), 0x4B37);
}

static void test_read_request_vector(void) {
    const uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    EXPECT_EQ_INT(crc16_modbus(data, sizeof data), 0xCDC5);
}

static void test_empty_is_initial_value(void) {
    EXPECT_EQ_INT(crc16_modbus(NULL, 0), 0xFFFF);
}

int main(void) {
    test_ascii_vector();
    test_read_request_vector();
    test_empty_is_initial_value();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
