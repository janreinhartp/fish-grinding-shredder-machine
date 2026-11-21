#include "device/board.h"

#include <stddef.h>
#include <stdint.h>

#include "device/profile.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define UART_RX_BUFFER_BYTES 256
#define UART_FRAME_GAP_MS 5
#define UART_FRAME_MAX_BYTES 64

static int uart_port(void) {
    return device_profile()->uart.port;
}

static int board_write(void *ctx, const uint8_t *data, size_t len) {
    (void)ctx;
    uart_port_t port = (uart_port_t)uart_port();
    int written = uart_write_bytes(port, data, len);
    if (written < 0 || (size_t)written != len) {
        return -1;
    }
    if (uart_wait_tx_done(port, pdMS_TO_TICKS(100)) != ESP_OK) {
        return -1;
    }
    /* Discard anything that arrived while we were transmitting (echo, noise). */
    if (uart_flush_input(port) != ESP_OK) {
        return -1;
    }
    return 0;
}

static int board_read(void *ctx, uint8_t *data, size_t max_len, uint32_t timeout_ms) {
    (void)ctx;
    uart_port_t port = (uart_port_t)uart_port();
    size_t limit = max_len < UART_FRAME_MAX_BYTES ? max_len : UART_FRAME_MAX_BYTES;
    if (data == NULL || limit == 0) {
        return -1;
    }

    int got = uart_read_bytes(port, data, 1, pdMS_TO_TICKS(timeout_ms));
    if (got < 0) {
        return -1;
    }
    if (got == 0) {
        return 0;
    }

    size_t total = 1;
    while (total < limit) {
        int n = uart_read_bytes(port, data + total, limit - total, pdMS_TO_TICKS(UART_FRAME_GAP_MS));
        if (n < 0) {
            return -1;
        }
        if (n == 0) {
            break;
        }
        total += (size_t)n;
    }
    return (int)total;
}

static modbus_link_t s_link;

esp_err_t board_modbus_init(void) {
    const profile_uart_t *cfg = &device_profile()->uart;
    uart_port_t port = (uart_port_t)cfg->port;

    uart_parity_t parity = UART_PARITY_DISABLE;
    if (cfg->parity == PROFILE_PARITY_EVEN) {
        parity = UART_PARITY_EVEN;
    } else if (cfg->parity == PROFILE_PARITY_ODD) {
        parity = UART_PARITY_ODD;
    }

    const uart_config_t uart_config = {
        .baud_rate = (int)cfg->baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(port, UART_RX_BUFFER_BYTES, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_param_config(port, &uart_config);
    if (err != ESP_OK) {
        return err;
    }

    int rts_pin = cfg->use_de ? cfg->de_pin : UART_PIN_NO_CHANGE;
    err = uart_set_pin(port, cfg->tx_pin, cfg->rx_pin, rts_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_mode(port, cfg->use_de ? UART_MODE_RS485_HALF_DUPLEX : UART_MODE_UART);
    if (err != ESP_OK) {
        return err;
    }

    s_link = (modbus_link_t){
        .write = board_write,
        .read = board_read,
        .ctx = NULL,
        .timeout_ms = device_profile()->timing.timeout_ms,
    };
    return ESP_OK;
}

const modbus_link_t *board_modbus_link(void) {
    return &s_link;
}
