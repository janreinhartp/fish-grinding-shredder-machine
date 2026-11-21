#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERR_TIMEOUT,
    MODBUS_ERR_CRC,
    MODBUS_ERR_SLAVE,
    MODBUS_ERR_FUNCTION,
    MODBUS_ERR_EXCEPTION,
    MODBUS_ERR_TRANSPORT,
    MODBUS_ERR_LENGTH,
    MODBUS_ERR_ARGUMENT,
} modbus_status_t;

typedef struct {
    int (*write)(void *ctx, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint8_t *data, size_t max_len, uint32_t timeout_ms);
    void *ctx;
    uint32_t timeout_ms;
} modbus_link_t;

modbus_status_t modbus_read_registers(const modbus_link_t *link, uint8_t slave, uint8_t function,
                                      uint16_t address, uint16_t count, uint16_t *regs);
modbus_status_t modbus_write_coils(const modbus_link_t *link, uint8_t slave, uint16_t address,
                                   uint16_t count, uint8_t coil_byte);
modbus_status_t modbus_write_single_coil(const modbus_link_t *link, uint8_t slave, uint16_t address,
                                         bool on);
