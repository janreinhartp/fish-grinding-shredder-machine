#include "hmi/display.h"

#include <stdint.h>

#include "device/profile.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"

static const char *TAG = "display";

#define PANEL_PCLK_HZ (20 * 1000 * 1000)
#define DRAW_BUFFER_LINES 24
#define BYTES_PER_PIXEL 2

static esp_lcd_panel_handle_t s_panel;
static lv_display_t *s_display;

static uint32_t lvgl_tick_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Runs when the panel I/O finishes sending a colour transaction. */
static bool on_color_trans_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *event, void *ctx) {
    (void)io;
    (void)event;
    lv_display_flush_ready((lv_display_t *)ctx);
    return false;
}

/* Does not call lv_display_flush_ready; on_color_trans_done does. */
static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    uint32_t width = (uint32_t)(area->x2 - area->x1 + 1);
    uint32_t height = (uint32_t)(area->y2 - area->y1 + 1);
    /* LVGL renders little-endian RGB565; the panel expects the high byte first. */
    lv_draw_sw_rgb565_swap(px_map, width * height);
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px_map);
}

static esp_err_t spi_host_from_profile(int host, spi_host_device_t *out) {
    if (host == 2) {
        *out = SPI2_HOST;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t display_init(void) {
    const profile_panel_t *cfg = &device_profile()->panel;
    const int h_res = cfg->width;
    const int v_res = cfg->height;
    const size_t buffer_bytes = (size_t)h_res * DRAW_BUFFER_LINES * BYTES_PER_PIXEL;

    spi_host_device_t host;
    esp_err_t err = spi_host_from_profile(cfg->spi_host, &host);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "unsupported SPI host %d", cfg->spi_host);
        return err;
    }

    /* The backlight starts off so the panel does not flash garbage during init. */
    const gpio_config_t backlight = {
        .pin_bit_mask = 1ULL << cfg->backlight_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&backlight);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level(cfg->backlight_pin, cfg->backlight_active_high ? 0 : 1);

    spi_bus_config_t bus = ILI9341_PANEL_BUS_SPI_CONFIG(cfg->sclk_pin, cfg->mosi_pin, (int)buffer_bytes);
    err = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        return err;
    }

    lv_init();
    lv_tick_set_cb(lvgl_tick_ms);

    s_display = lv_display_create(h_res, v_res);
    if (s_display == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_lcd_panel_io_spi_config_t io_config = ILI9341_PANEL_IO_SPI_CONFIG(cfg->cs_pin, cfg->dc_pin, on_color_trans_done, s_display);
    io_config.pclk_hz = PANEL_PCLK_HZ;
    io_config.spi_mode = 0;
    esp_lcd_panel_io_handle_t io = NULL;
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host, &io_config, &io);
    if (err != ESP_OK) {
        return err;
    }

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = cfg->rst_pin,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_ili9341(io, &panel_config, &s_panel);
    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, cfg->swap_xy));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, cfg->mirror_x, cfg->mirror_y));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    void *buf1 = heap_caps_malloc(buffer_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void *buf2 = heap_caps_malloc(buffer_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf1 == NULL || buf2 == NULL) {
        ESP_LOGE(TAG, "draw buffer allocation failed");
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return ESP_ERR_NO_MEM;
    }

    lv_display_set_color_format(s_display, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(s_display, buf1, buf2, (uint32_t)buffer_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(s_display, s_panel);
    lv_display_set_flush_cb(s_display, flush_cb);

    gpio_set_level(cfg->backlight_pin, cfg->backlight_active_high ? 1 : 0);
    return ESP_OK;
}
