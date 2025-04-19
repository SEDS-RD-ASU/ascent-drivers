#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include "driver/spi_common.h"
#include "esp_err.h"

esp_err_t spi_manager_init(spi_host_device_t host_id, int mosi_io_num, int miso_io_num, int sclk_io_num);
esp_err_t spi_manager_deinit(spi_host_device_t host_id);

#endif