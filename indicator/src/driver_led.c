#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include <rom/ets_sys.h>


#define LED_PIN 21
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 8  // 80MHz / 8 = 10MHz → 100ns ticks
#define RESET_US 300

#define WS2812_T0H_NS 400
#define WS2812_T0L_NS 850
#define WS2812_T1H_NS 800
#define WS2812_T1L_NS 450

typedef struct { uint8_t g, r, b; } led_color_t;

static const led_color_t RED   = {0, 255, 0};
static const led_color_t GREEN = {255, 0, 0};
static const led_color_t BLUE  = {0, 0, 255};
static const led_color_t WHITE = {255, 255, 255};
static const led_color_t BLACK = {0, 0, 0};

static uint32_t t0h_ticks, t0l_ticks, t1h_ticks, t1l_ticks;

static void ws2812_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
                               size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    const rmt_item32_t bit0 = {{{ t0h_ticks, 1, t0l_ticks, 0 }}};
    const rmt_item32_t bit1 = {{{ t1h_ticks, 1, t1l_ticks, 0 }}};

    size_t size = 0, num = 0;
    const uint8_t *data = (const uint8_t *)src;

    while (size < src_size && num < wanted_num) {
        for (int i = 7; i >= 0; i--) {
            dest[num++] = (*data & (1 << i)) ? bit1 : bit0;
        }
        data++;
        size++;
    }

    *translated_size = size;
    *item_num = num;
}

void led_init(void) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_PIN, RMT_CHANNEL);
    config.clk_div = RMT_CLK_DIV;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // Compute tick timing from nanoseconds (10MHz = 100ns per tick)
    float ratio = (float) (APB_CLK_FREQ / 10000000); // 1 tick = 100ns
    t0h_ticks = (uint32_t)(WS2812_T0H_NS * ratio);
    t0l_ticks = (uint32_t)(WS2812_T0L_NS * ratio);
    t1h_ticks = (uint32_t)(WS2812_T1H_NS * ratio);
    t1l_ticks = (uint32_t)(WS2812_T1L_NS * ratio);

    ESP_ERROR_CHECK(rmt_translator_init(RMT_CHANNEL, ws2812_rmt_adapter));
}

void send_rgb(led_color_t color) {
    uint8_t grb[3] = { color.g, color.r, color.b };
    ESP_ERROR_CHECK(rmt_write_sample(RMT_CHANNEL, grb, sizeof(grb), true));
    rmt_wait_tx_done(RMT_CHANNEL, pdMS_TO_TICKS(RESET_US));
}

void send_reset() {
    gpio_set_level(LED_PIN, 0);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    ets_delay_us(RESET_US);  // delay ≥ 50us (we use 300us)
}

void send_color(const char *name) {
    if (strcmp(name, "red") == 0) {
        send_rgb(RED);
    } else if (strcmp(name, "green") == 0) {
        send_rgb(GREEN);
    } else if (strcmp(name, "blue") == 0) {
        send_rgb(BLUE);
    } else if (strcmp(name, "white") == 0) {
        send_rgb(WHITE);
    } else {
        send_rgb(BLACK);
    }
}