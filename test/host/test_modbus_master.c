#include "test_harness.h"
#include "common/crc16.h"
#include "modbus/modbus_master.h"

#include <string.h>

int g_failures = 0;

typedef struct {
    uint8_t tx[64];
    size_t tx_len;
    int write_calls;
    int write_result;
    uint8_t rx[64];
    size_t rx_len;
    int read_result_override;
    bool use_override;
} fake_t;

static int fake_write(void *ctx, const uint8_t *data, size_t len) {
    fake_t *f = (fake_t *)ctx;
    f->write_calls++;
    if (f->write_result != 0) {
        return f->write_result;
    }
    memcpy(f->tx, data, len);
    f->tx_len = len;
    return 0;
}

static int fake_read(void *ctx, uint8_t *data, size_t max_len, uint32_t timeout_ms) {
    fake_t *f = (fake_t *)ctx;
    (void)timeout_ms;
    if (f->use_override) {
        return f->read_result_override;
    }
    size_t n = f->rx_len < max_len ? f->rx_len : max_len;
    memcpy(data, f->rx, n);
    return (int)n;
}

static void fake_init(fake_t *f, const uint8_t *rx, size_t rx_len) {
    memset(f, 0, sizeof *f);
    if (rx_len > 0) {
        memcpy(f->rx, rx, rx_len);
    }
    f->rx_len = rx_len;
}

static modbus_link_t make_link(fake_t *f) {
    modbus_link_t link = {fake_write, fake_read, f, 100};
    return link;
}

/* Expect the recorded TX frame to be `body` plus CRC (low byte first). */
static void expect_tx(const fake_t *f, const uint8_t *body, size_t body_len) {
    uint16_t crc = crc16_modbus(body, body_len);
    EXPECT_EQ_INT(f->tx_len, body_len + 2);
    EXPECT(memcmp(f->tx, body, body_len) == 0);
    EXPECT_EQ_INT(f->tx[body_len], crc & 0xFF);
    EXPECT_EQ_INT(f->tx[body_len + 1], crc >> 8);
}

static const uint8_t k_read_reply[] = {0x01, 0x03, 0x04, 0x00, 0x00, 0x67, 0x20, 0xD1, 0xDB};

static void test_read_ok(void) {
    fake_t f;
    fake_init(&f, k_read_reply, sizeof k_read_reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0xAAAA, 0xBBBB};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_OK);
    const uint8_t body[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
    expect_tx(&f, body, sizeof body);
    EXPECT_EQ_INT(regs[0], 0x0000);
    EXPECT_EQ_INT(regs[1], 0x6720);
}

static void test_read_function_04(void) {
    uint8_t reply[9] = {0x01, 0x04, 0x04, 0x00, 0x00, 0x67, 0x20};
    uint16_t crc = crc16_modbus(reply, 7);
    reply[7] = crc & 0xFF;
    reply[8] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x04, 0, 2, regs), MODBUS_OK);
    EXPECT_EQ_INT(f.tx[1], 0x04);
    EXPECT_EQ_INT(regs[1], 0x6720);
}

static void test_read_bad_crc(void) {
    uint8_t reply[sizeof k_read_reply];
    memcpy(reply, k_read_reply, sizeof reply);
    reply[sizeof reply - 1] ^= 0xFF;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_CRC);
}

static void test_read_timeout(void) {
    fake_t f;
    fake_init(&f, NULL, 0);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_TIMEOUT);
}

static void test_read_exception(void) {
    const uint8_t reply[] = {0x01, 0x83, 0x02, 0xC0, 0xF1};
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_EXCEPTION);
}

static void test_read_short_frame(void) {
    const uint8_t reply[] = {0x01, 0x03, 0x04, 0x00};
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_LENGTH);
}

static void test_read_wrong_slave(void) {
    uint8_t reply[9] = {0x05, 0x03, 0x04, 0x00, 0x00, 0x67, 0x20};
    uint16_t crc = crc16_modbus(reply, 7);
    reply[7] = crc & 0xFF;
    reply[8] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_SLAVE);
}

static void test_read_wrong_function(void) {
    uint8_t reply[9] = {0x01, 0x04, 0x04, 0x00, 0x00, 0x67, 0x20};
    uint16_t crc = crc16_modbus(reply, 7);
    reply[7] = crc & 0xFF;
    reply[8] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_FUNCTION);
}

static void test_read_byte_count_mismatch(void) {
    /* Byte count 2 but caller asked for 2 registers (needs 4). Valid CRC. */
    uint8_t reply[7] = {0x01, 0x03, 0x02, 0x67, 0x20};
    uint16_t crc = crc16_modbus(reply, 5);
    reply[5] = crc & 0xFF;
    reply[6] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_LENGTH);
}

static void test_read_arguments(void) {
    fake_t f;
    fake_init(&f, k_read_reply, sizeof k_read_reply);
    modbus_link_t link = make_link(&f);
    uint16_t regs[9] = {0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 0, regs), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 9, regs), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x06, 0, 2, regs), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, NULL), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(modbus_read_registers(NULL, 1, 0x03, 0, 2, regs), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(f.write_calls, 0);
}

static void test_read_write_failure(void) {
    fake_t f;
    fake_init(&f, k_read_reply, sizeof k_read_reply);
    f.write_result = -1;
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_TRANSPORT);
}

static void test_read_negative_read(void) {
    fake_t f;
    fake_init(&f, k_read_reply, sizeof k_read_reply);
    f.use_override = true;
    f.read_result_override = -1;
    modbus_link_t link = make_link(&f);
    uint16_t regs[2] = {0, 0};
    EXPECT_EQ_INT(modbus_read_registers(&link, 1, 0x03, 0, 2, regs), MODBUS_ERR_TRANSPORT);
}

static void test_write_coils_ok(void) {
    const uint8_t reply[] = {0x02, 0x0F, 0x00, 0x00, 0x00, 0x03, 0x15, 0xF9};
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 3, 0x07), MODBUS_OK);
    const uint8_t body[] = {0x02, 0x0F, 0x00, 0x00, 0x00, 0x03, 0x01, 0x07};
    expect_tx(&f, body, sizeof body);
}

static void test_write_coils_bad_echo(void) {
    /* Echoes quantity 2 instead of 3, with a valid CRC. */
    uint8_t reply[8] = {0x02, 0x0F, 0x00, 0x00, 0x00, 0x02};
    uint16_t crc = crc16_modbus(reply, 6);
    reply[6] = crc & 0xFF;
    reply[7] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 3, 0x07), MODBUS_ERR_FUNCTION);
}

static void test_write_coils_bad_crc(void) {
    const uint8_t reply[] = {0x02, 0x0F, 0x00, 0x00, 0x00, 0x03, 0x15, 0xF8};
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 3, 0x07), MODBUS_ERR_CRC);
}

static void test_write_coils_timeout(void) {
    fake_t f;
    fake_init(&f, NULL, 0);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 3, 0x07), MODBUS_ERR_TIMEOUT);
}

static void test_write_coils_arguments(void) {
    fake_t f;
    fake_init(&f, NULL, 0);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 0, 0x07), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(modbus_write_coils(NULL, 2, 0, 3, 0x07), MODBUS_ERR_ARGUMENT);
    EXPECT_EQ_INT(f.write_calls, 0);
}

static void test_write_coils_write_failure(void) {
    fake_t f;
    fake_init(&f, NULL, 0);
    f.write_result = -1;
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_coils(&link, 2, 0, 3, 0x07), MODBUS_ERR_TRANSPORT);
}

static void test_write_single_coil_on(void) {
    uint8_t reply[8] = {0x02, 0x05, 0x00, 0x00, 0xFF, 0x00};
    uint16_t crc = crc16_modbus(reply, 6);
    reply[6] = crc & 0xFF;
    reply[7] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_single_coil(&link, 2, 0, true), MODBUS_OK);
    const uint8_t body[] = {0x02, 0x05, 0x00, 0x00, 0xFF, 0x00};
    expect_tx(&f, body, sizeof body);
}

static void test_write_single_coil_off(void) {
    uint8_t reply[8] = {0x02, 0x05, 0x00, 0x01, 0x00, 0x00};
    uint16_t crc = crc16_modbus(reply, 6);
    reply[6] = crc & 0xFF;
    reply[7] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_single_coil(&link, 2, 1, false), MODBUS_OK);
    const uint8_t body[] = {0x02, 0x05, 0x00, 0x01, 0x00, 0x00};
    expect_tx(&f, body, sizeof body);
}

static void test_write_single_coil_mismatch(void) {
    /* Echo says OFF though ON was sent; valid CRC. */
    uint8_t reply[8] = {0x02, 0x05, 0x00, 0x00, 0x00, 0x00};
    uint16_t crc = crc16_modbus(reply, 6);
    reply[6] = crc & 0xFF;
    reply[7] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_single_coil(&link, 2, 0, true), MODBUS_ERR_FUNCTION);
}

static void test_write_single_coil_exception(void) {
    uint8_t reply[5] = {0x02, 0x85, 0x02};
    uint16_t crc = crc16_modbus(reply, 3);
    reply[3] = crc & 0xFF;
    reply[4] = crc >> 8;
    fake_t f;
    fake_init(&f, reply, sizeof reply);
    modbus_link_t link = make_link(&f);
    EXPECT_EQ_INT(modbus_write_single_coil(&link, 2, 0, true), MODBUS_ERR_EXCEPTION);
}

int main(void) {
    test_read_ok();
    test_read_function_04();
    test_read_bad_crc();
    test_read_timeout();
    test_read_exception();
    test_read_short_frame();
    test_read_wrong_slave();
    test_read_wrong_function();
    test_read_byte_count_mismatch();
    test_read_arguments();
    test_read_write_failure();
    test_read_negative_read();
    test_write_coils_ok();
    test_write_coils_bad_echo();
    test_write_coils_bad_crc();
    test_write_coils_timeout();
    test_write_coils_arguments();
    test_write_coils_write_failure();
    test_write_single_coil_on();
    test_write_single_coil_off();
    test_write_single_coil_mismatch();
    test_write_single_coil_exception();
    if (g_failures != 0) {
        fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
