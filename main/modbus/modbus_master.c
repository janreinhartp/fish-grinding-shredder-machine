#include "modbus/modbus_master.h"

#include "common/crc16.h"

#include <string.h>

#define MODBUS_RX_MAX 64
#define MODBUS_MAX_REGS 8

static void put_crc(uint8_t *frame, size_t body_len) {
    uint16_t crc = crc16_modbus(frame, body_len);
    frame[body_len] = (uint8_t)(crc & 0xFF);
    frame[body_len + 1] = (uint8_t)(crc >> 8);
}

static bool link_valid(const modbus_link_t *link) {
    return link != NULL && link->write != NULL && link->read != NULL;
}

/* Send a frame and read the reply. Returns MODBUS_OK with *rx_len set when a
 * frame of at least 5 bytes with a valid CRC arrived. */
static modbus_status_t transact(const modbus_link_t *link, const uint8_t *tx, size_t tx_len,
                                uint8_t *rx, size_t *rx_len) {
    if (link->write(link->ctx, tx, tx_len) != 0) {
        return MODBUS_ERR_TRANSPORT;
    }
    int n = link->read(link->ctx, rx, MODBUS_RX_MAX, link->timeout_ms);
    if (n < 0) {
        return MODBUS_ERR_TRANSPORT;
    }
    if (n == 0) {
        return MODBUS_ERR_TIMEOUT;
    }
    if (n < 5 || n > MODBUS_RX_MAX) {
        return MODBUS_ERR_LENGTH;
    }
    uint16_t crc = crc16_modbus(rx, (size_t)n - 2);
    if (rx[n - 2] != (uint8_t)(crc & 0xFF) || rx[n - 1] != (uint8_t)(crc >> 8)) {
        return MODBUS_ERR_CRC;
    }
    *rx_len = (size_t)n;
    return MODBUS_OK;
}

/* Common checks for slave and function (including the exception bit). */
static modbus_status_t check_header(const uint8_t *rx, uint8_t slave, uint8_t function) {
    if (rx[0] != slave) {
        return MODBUS_ERR_SLAVE;
    }
    if ((rx[1] & 0x80) != 0) {
        return MODBUS_ERR_EXCEPTION;
    }
    if (rx[1] != function) {
        return MODBUS_ERR_FUNCTION;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_read_registers(const modbus_link_t *link, uint8_t slave, uint8_t function,
                                      uint16_t address, uint16_t count, uint16_t *regs) {
    if (!link_valid(link) || regs == NULL || count == 0 || count > MODBUS_MAX_REGS ||
        (function != 0x03 && function != 0x04)) {
        return MODBUS_ERR_ARGUMENT;
    }
    uint8_t tx[8] = {slave, function, (uint8_t)(address >> 8), (uint8_t)address,
                     (uint8_t)(count >> 8), (uint8_t)count};
    put_crc(tx, 6);
    uint8_t rx[MODBUS_RX_MAX];
    size_t rx_len = 0;
    modbus_status_t st = transact(link, tx, sizeof tx, rx, &rx_len);
    if (st != MODBUS_OK) {
        return st;
    }
    st = check_header(rx, slave, function);
    if (st != MODBUS_OK) {
        return st;
    }
    if (rx[2] != count * 2 || rx_len != (size_t)(5 + count * 2)) {
        return MODBUS_ERR_LENGTH;
    }
    for (uint16_t i = 0; i < count; i++) {
        regs[i] = (uint16_t)((rx[3 + 2 * i] << 8) | rx[4 + 2 * i]);
    }
    return MODBUS_OK;
}

/* Write functions 0x0F and 0x05 echo slave, function, address and a 16-bit
 * field in an 8-byte reply. */
static modbus_status_t expect_echo(const modbus_link_t *link, const uint8_t *tx, size_t tx_len) {
    uint8_t rx[MODBUS_RX_MAX];
    size_t rx_len = 0;
    modbus_status_t st = transact(link, tx, tx_len, rx, &rx_len);
    if (st != MODBUS_OK) {
        return st;
    }
    st = check_header(rx, tx[0], tx[1]);
    if (st != MODBUS_OK) {
        return st;
    }
    if (rx_len != 8 || memcmp(rx, tx, 6) != 0) {
        return MODBUS_ERR_FUNCTION;
    }
    return MODBUS_OK;
}

modbus_status_t modbus_write_coils(const modbus_link_t *link, uint8_t slave, uint16_t address,
                                   uint16_t count, uint8_t coil_byte) {
    if (!link_valid(link) || count == 0 || count > 8) {
        return MODBUS_ERR_ARGUMENT;
    }
    uint8_t tx[10] = {slave, 0x0F, (uint8_t)(address >> 8), (uint8_t)address,
                      (uint8_t)(count >> 8), (uint8_t)count, 0x01, coil_byte};
    put_crc(tx, 8);
    return expect_echo(link, tx, sizeof tx);
}

modbus_status_t modbus_write_single_coil(const modbus_link_t *link, uint8_t slave, uint16_t address,
                                         bool on) {
    if (!link_valid(link)) {
        return MODBUS_ERR_ARGUMENT;
    }
    uint8_t tx[8] = {slave, 0x05, (uint8_t)(address >> 8), (uint8_t)address,
                     on ? 0xFF : 0x00, 0x00};
    put_crc(tx, 6);
    return expect_echo(link, tx, sizeof tx);
}
