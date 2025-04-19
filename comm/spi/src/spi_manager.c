#include "spi_manager.h"

esp_err_t spi_manager_init(spi_host_device_t host_id, int mosi_io_num, int miso_io_num, int sclk_io_num)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = mosi_io_num,
        .miso_io_num = miso_io_num,
        .sclk_io_num = sclk_io_num,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // .max_transfer_sz = 0 
        .max_transfer_sz = 8192
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(host_id, &bus_config, SPI_DMA_CH_AUTO);

    return ret;
}

esp_err_t spi_manager_deinit(spi_host_device_t host_id)
{
    return spi_bus_free(host_id);
}