#include "manual_spi_bus.h"
#include <rom/ets_sys.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Static variables for pin configuration
static int pin_miso;
static int pin_mosi;
static int pin_sclk;
static int pin_cs;

esp_err_t spi_init(int miso, int mosi, int sclk, int cs) {
    pin_miso = miso;
    pin_mosi = mosi;
    pin_sclk = sclk;
    pin_cs = cs;

    gpio_set_direction(pin_cs, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_sclk, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_mosi, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_miso, GPIO_MODE_INPUT);

    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

void spi_write_byte(uint8_t data) {
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(pin_mosi, (data >> i) & 0x01); // Set MOSI
        gpio_set_level(pin_sclk, 1); // Clock high
        ets_delay_us(1);
        gpio_set_level(pin_sclk, 0); // Clock low
    }
}

uint8_t spi_read_byte(void) {
    uint8_t data = 0;
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(pin_sclk, 1); // Clock high
        ets_delay_us(1);
        if (gpio_get_level(pin_miso)) { // Read MISO
            data |= (1 << i);
        }
        gpio_set_level(pin_sclk, 0); // Clock low
    }

    return data;
}

uint8_t spi_exchange_byte(uint8_t tx) {
    uint8_t rx = 0;
    for (int i = 7; i >= 0; i--) { // MSB first
        gpio_set_level(pin_mosi, (tx >> i) & 0x01); // Set MOSI

        gpio_set_level(pin_sclk, 1); // Clock high
        ets_delay_us(1);

        if (gpio_get_level(pin_miso)) { // Read MISO
            rx |= (1 << i);
        }

        gpio_set_level(pin_sclk, 0); // Clock low
    }

    return rx;
}

void spi_begin(void) {
    gpio_set_level(pin_cs, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(pin_cs, 0);
}

void spi_end(void) {
    gpio_set_level(pin_cs, 1);
}