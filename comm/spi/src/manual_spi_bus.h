#ifndef MANUAL_SPI_BUS_H
#define MANUAL_SPI_BUS_H

#include <string.h>
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief Initialize the manual SPI bus
 * 
 * @param miso GPIO pin number for MISO
 * @param mosi GPIO pin number for MOSI
 * @param sclk GPIO pin number for SCLK
 * @param cs GPIO pin number for CS
 * @return esp_err_t ESP_OK on success
 */
esp_err_t spi_init(int miso, int mosi, int sclk, int cs);

/**
 * @brief Write a byte to the SPI bus
 * 
 * @param data Byte to write
 */
void spi_write_byte(uint8_t data);

/**
 * @brief Read a byte from the SPI bus
 * 
 * @return uint8_t Byte read from the bus
 */
uint8_t spi_read_byte(void);

/**
 * @brief Exchange a byte on the SPI bus
 * 
 * @param tx Byte to transmit
 * @return uint8_t Byte received
 */
uint8_t spi_exchange_byte(uint8_t tx);

/**
 * @brief Begin an SPI transaction by asserting CS
 */
void spi_begin(void);

/**
 * @brief End an SPI transaction by deasserting CS
 */
void spi_end(void);

#endif /* MANUAL_SPI_BUS_H */