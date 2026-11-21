#include "hmi/encoder_hw.h"

#include "device/profile.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define SAMPLE_QUEUE_DEPTH 32

static QueueHandle_t s_queue;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static encoder_sample_t s_latest;

static encoder_sample_t read_pins(void) {
    const profile_encoder_t *cfg = &device_profile()->encoder;
    encoder_sample_t sample = {
        .a = (uint8_t)gpio_get_level(cfg->a_pin),
        .b = (uint8_t)gpio_get_level(cfg->b_pin),
        .sw = (uint8_t)gpio_get_level(cfg->switch_pin),
        .ms = (uint32_t)(esp_timer_get_time() / 1000),
    };
    return sample;
}

static void encoder_isr(void *arg) {
    (void)arg;
    encoder_sample_t sample = read_pins();

    portENTER_CRITICAL_ISR(&s_mux);
    s_latest = sample;
    portEXIT_CRITICAL_ISR(&s_mux);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_queue, &sample, &woken); /* a full queue drops the sample */
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t encoder_hw_init(void) {
    const profile_encoder_t *cfg = &device_profile()->encoder;

    s_queue = xQueueCreate(SAMPLE_QUEUE_DEPTH, sizeof(encoder_sample_t));
    if (s_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const int pins[] = {cfg->a_pin, cfg->b_pin, cfg->switch_pin};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        const gpio_config_t io = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) {
            return err;
        }
    }

    s_latest = read_pins();

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        err = gpio_isr_handler_add(pins[i], encoder_isr, NULL);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

bool encoder_hw_take(encoder_sample_t *out) {
    if (out == NULL || s_queue == NULL) {
        return false;
    }
    return xQueueReceive(s_queue, out, 0) == pdTRUE;
}

encoder_sample_t encoder_hw_latest(void) {
    encoder_sample_t sample;
    portENTER_CRITICAL(&s_mux);
    sample = s_latest;
    portEXIT_CRITICAL(&s_mux);
    return sample;
}
