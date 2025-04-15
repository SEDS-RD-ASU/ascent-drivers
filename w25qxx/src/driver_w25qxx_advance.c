/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_w25qxx_advance.c
 * @brief     driver w25qxx advance source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_w25qxx_advance.h"
#include "manual_spi_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "W25QXX_ADV";

// Static variables for SPI configuration
static int spi_miso;
static int spi_mosi;
static int spi_sclk;
static int spi_cs;
static bool quad_mode_enabled;

// W25QXX Commands
#define W25QXX_CMD_WRITE_ENABLE           0x06
#define W25QXX_CMD_WRITE_DISABLE          0x04
#define W25QXX_CMD_READ_STATUS_REG1       0x05
#define W25QXX_CMD_READ_STATUS_REG2       0x35
#define W25QXX_CMD_READ_STATUS_REG3       0x15
#define W25QXX_CMD_WRITE_STATUS_REG1      0x01
#define W25QXX_CMD_WRITE_STATUS_REG2      0x31
#define W25QXX_CMD_WRITE_STATUS_REG3      0x11
#define W25QXX_CMD_READ_DATA              0x03
#define W25QXX_CMD_FAST_READ              0x0B
#define W25QXX_CMD_FAST_READ_DUAL_OUTPUT  0x3B
#define W25QXX_CMD_FAST_READ_QUAD_OUTPUT  0x6B
#define W25QXX_CMD_PAGE_PROGRAM           0x02
#define W25QXX_CMD_SECTOR_ERASE           0x20
#define W25QXX_CMD_BLOCK_ERASE_32K        0x52
#define W25QXX_CMD_BLOCK_ERASE_64K        0xD8
#define W25QXX_CMD_CHIP_ERASE             0xC7
#define W25QXX_CMD_POWER_DOWN             0xB9
#define W25QXX_CMD_RELEASE_POWER_DOWN     0xAB
#define W25QXX_CMD_READ_ID                0x90
#define W25QXX_CMD_ENABLE_RESET           0x66
#define W25QXX_CMD_RESET_DEVICE           0x99

// Status register bits
#define W25QXX_STATUS_BUSY               0x01
#define W25QXX_STATUS_WEL                0x02

static void wait_busy(void)
{
    uint8_t status;
    do {
        spi_begin();
        spi_write_byte(W25QXX_CMD_READ_STATUS_REG1);
        status = spi_read_byte();
        spi_end();
    } while (status & W25QXX_STATUS_BUSY);
}

static void write_enable(void)
{
    spi_begin();
    spi_write_byte(W25QXX_CMD_WRITE_ENABLE);
    spi_end();
    
    // Wait for WEL bit to be set
    uint8_t status;
    do {
        spi_begin();
        spi_write_byte(W25QXX_CMD_READ_STATUS_REG1);
        status = spi_read_byte();
        spi_end();
    } while (!(status & W25QXX_STATUS_WEL));
}

uint8_t w25qxx_advance_init(int miso, int mosi, int sclk, int cs, bool quad_enable)
{
    spi_miso = miso;
    spi_mosi = mosi;
    spi_sclk = sclk;
    spi_cs = cs;
    quad_mode_enabled = quad_enable;

    esp_err_t ret = spi_init(miso, mosi, sclk, cs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return 1;
    }

    // Reset the device
    spi_begin();
    spi_write_byte(W25QXX_CMD_ENABLE_RESET);
    spi_end();
    
    spi_begin();
    spi_write_byte(W25QXX_CMD_RESET_DEVICE);
    spi_end();

    vTaskDelay(pdMS_TO_TICKS(50));  // Wait for reset to complete

    if (quad_enable) {
        // Enable Quad mode in Status Register 2
        write_enable();
        spi_begin();
        spi_write_byte(W25QXX_CMD_WRITE_STATUS_REG2);
        spi_write_byte(0x02);  // Set QUAD bit
        spi_end();
        wait_busy();
    }

    ESP_LOGI(TAG, "W25QXX initialized with MISO=%d, MOSI=%d, SCLK=%d, CS=%d, QUAD=%d",
             miso, mosi, sclk, cs, quad_enable);
    return 0;
}

uint8_t w25qxx_advance_power_down(void)
{
    spi_begin();
    spi_write_byte(W25QXX_CMD_POWER_DOWN);
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_wake_up(void)
{
    spi_begin();
    spi_write_byte(W25QXX_CMD_RELEASE_POWER_DOWN);
    spi_end();
    vTaskDelay(pdMS_TO_TICKS(3));  // tRES1 = 3us
    return 0;
}

uint8_t w25qxx_advance_chip_erase(void)
{
    write_enable();
    spi_begin();
    spi_write_byte(W25QXX_CMD_CHIP_ERASE);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_get_id(uint8_t *manufacturer, uint8_t *device_id)
{
    if (!manufacturer || !device_id) {
        ESP_LOGE(TAG, "NULL pointer provided");
        return 1;
    }

    spi_begin();
    spi_write_byte(W25QXX_CMD_READ_ID);
    spi_write_byte(0x00);  // Dummy byte
    spi_write_byte(0x00);  // Dummy byte
    spi_write_byte(0x00);  // Dummy byte
    *manufacturer = spi_read_byte();
    *device_id = spi_read_byte();
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_write(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!data) {
        ESP_LOGE(TAG, "NULL data pointer");
        return 1;
    }

    uint32_t page_offset = addr & 0xFF;
    uint32_t bytes_remaining = len;
    uint32_t current_addr = addr;
    uint32_t current_data_index = 0;

    while (bytes_remaining > 0) {
        uint32_t bytes_to_write = 256 - page_offset;
        if (bytes_to_write > bytes_remaining) {
            bytes_to_write = bytes_remaining;
        }

        write_enable();
        
        spi_begin();
        spi_write_byte(W25QXX_CMD_PAGE_PROGRAM);
        spi_write_byte((current_addr >> 16) & 0xFF);
        spi_write_byte((current_addr >> 8) & 0xFF);
        spi_write_byte(current_addr & 0xFF);
        
        for (uint32_t i = 0; i < bytes_to_write; i++) {
            spi_write_byte(data[current_data_index + i]);
        }
        
        spi_end();
        wait_busy();

        bytes_remaining -= bytes_to_write;
        current_addr += bytes_to_write;
        current_data_index += bytes_to_write;
        page_offset = 0;  // Only first page might be partial
    }

    return 0;
}

uint8_t w25qxx_advance_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!data) {
        ESP_LOGE(TAG, "NULL data pointer");
        return 1;
    }

    if (quad_mode_enabled) {
        return w25qxx_advance_fast_read(addr, data, len);
    }

    spi_begin();
    spi_write_byte(W25QXX_CMD_READ_DATA);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    
    for (uint32_t i = 0; i < len; i++) {
        data[i] = spi_read_byte();
    }
    
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_fast_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!data) {
        ESP_LOGE(TAG, "NULL data pointer");
        return 1;
    }

    uint8_t cmd = quad_mode_enabled ? W25QXX_CMD_FAST_READ_QUAD_OUTPUT : W25QXX_CMD_FAST_READ;

    spi_begin();
    spi_write_byte(cmd);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_write_byte(0);  // Dummy byte
    
    for (uint32_t i = 0; i < len; i++) {
        data[i] = spi_read_byte();
    }
    
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_page_program(uint32_t addr, uint8_t *data, uint16_t len)
{
    if (!data || len > 256) {
        ESP_LOGE(TAG, "Invalid parameters");
        return 1;
    }

    write_enable();
    
    spi_begin();
    spi_write_byte(W25QXX_CMD_PAGE_PROGRAM);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    
    for (uint16_t i = 0; i < len; i++) {
        spi_write_byte(data[i]);
    }
    
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_sector_erase_4k(uint32_t addr)
{
    write_enable();
    
    spi_begin();
    spi_write_byte(W25QXX_CMD_SECTOR_ERASE);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_end();
    
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_block_erase_32k(uint32_t addr)
{
    write_enable();
    
    spi_begin();
    spi_write_byte(W25QXX_CMD_BLOCK_ERASE_32K);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_end();
    
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_block_erase_64k(uint32_t addr)
{
    write_enable();
    
    spi_begin();
    spi_write_byte(W25QXX_CMD_BLOCK_ERASE_64K);
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_end();
    
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_get_status1(uint8_t *status)
{
    if (!status) {
        ESP_LOGE(TAG, "NULL status pointer");
        return 1;
    }

    spi_begin();
    spi_write_byte(W25QXX_CMD_READ_STATUS_REG1);
    *status = spi_read_byte();
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_get_status2(uint8_t *status)
{
    if (!status) {
        ESP_LOGE(TAG, "NULL status pointer");
        return 1;
    }

    spi_begin();
    spi_write_byte(W25QXX_CMD_READ_STATUS_REG2);
    *status = spi_read_byte();
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_get_status3(uint8_t *status)
{
    if (!status) {
        ESP_LOGE(TAG, "NULL status pointer");
        return 1;
    }

    spi_begin();
    spi_write_byte(W25QXX_CMD_READ_STATUS_REG3);
    *status = spi_read_byte();
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_set_status1(uint8_t status)
{
    write_enable();
    spi_begin();
    spi_write_byte(W25QXX_CMD_WRITE_STATUS_REG1);
    spi_write_byte(status);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_set_status2(uint8_t status)
{
    write_enable();
    spi_begin();
    spi_write_byte(W25QXX_CMD_WRITE_STATUS_REG2);
    spi_write_byte(status);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_set_status3(uint8_t status)
{
    write_enable();
    spi_begin();
    spi_write_byte(W25QXX_CMD_WRITE_STATUS_REG3);
    spi_write_byte(status);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_individual_block_lock(uint32_t addr)
{
    write_enable();
    spi_begin();
    spi_write_byte(0x36);  // Individual Block Lock command
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_individual_block_unlock(uint32_t addr)
{
    write_enable();
    spi_begin();
    spi_write_byte(0x39);  // Individual Block Unlock command
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    spi_end();
    wait_busy();
    return 0;
}

uint8_t w25qxx_advance_read_block_lock(uint32_t addr, uint8_t *locked)
{
    if (!locked) {
        ESP_LOGE(TAG, "NULL locked pointer");
        return 1;
    }

    spi_begin();
    spi_write_byte(0x3D);  // Read Block Lock command
    spi_write_byte((addr >> 16) & 0xFF);
    spi_write_byte((addr >> 8) & 0xFF);
    spi_write_byte(addr & 0xFF);
    *locked = spi_read_byte();
    spi_end();
    return 0;
}

uint8_t w25qxx_advance_deinit(void)
{
    return 0;
}