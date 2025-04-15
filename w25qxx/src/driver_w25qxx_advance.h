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
 * @file      driver_w25qxx_advance.h
 * @brief     driver w25qxx advance header file
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

#ifndef DRIVER_W25QXX_ADVANCE_H
#define DRIVER_W25QXX_ADVANCE_H

#include "driver_w25qxx_interface.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @addtogroup w25qxx_example_driver
 * @{
 */

/**
 * @brief Initialize the W25QXX flash memory in advanced mode
 * 
 * @param miso GPIO pin number for MISO
 * @param mosi GPIO pin number for MOSI
 * @param sclk GPIO pin number for SCLK
 * @param cs GPIO pin number for CS
 * @param quad_enable Enable quad SPI mode if true
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_init(int miso, int mosi, int sclk, int cs, bool quad_enable);

/**
 * @brief Power down the flash memory to save power
 * 
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_power_down(void);

/**
 * @brief Wake up the flash memory from power down mode
 * 
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_wake_up(void);

/**
 * @brief Erase the entire chip
 * 
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_chip_erase(void);

/**
 * @brief Get the manufacturer and device ID
 * 
 * @param manufacturer Pointer to store manufacturer ID
 * @param device_id Pointer to store device ID
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_get_id(uint8_t *manufacturer, uint8_t *device_id);

/**
 * @brief Write data to flash memory with advanced features
 * 
 * @param addr Address to write to
 * @param data Pointer to data buffer
 * @param len Length of data to write
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_write(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Read data from flash memory with advanced features
 * 
 * @param addr Address to read from
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_read(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Program a single page (256 bytes or less)
 * 
 * @param addr Address to program (must be page-aligned)
 * @param data Pointer to data buffer
 * @param len Length of data (max 256 bytes)
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_page_program(uint32_t addr, uint8_t *data, uint16_t len);

/**
 * @brief Erase a 4KB sector
 * 
 * @param addr Address within the sector to erase
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_sector_erase_4k(uint32_t addr);

/**
 * @brief Erase a 32KB block
 * 
 * @param addr Address within the block to erase
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_block_erase_32k(uint32_t addr);

/**
 * @brief Erase a 64KB block
 * 
 * @param addr Address within the block to erase
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_block_erase_64k(uint32_t addr);

/**
 * @brief Fast read operation for higher performance
 * 
 * @param addr Address to read from
 * @param data Pointer to data buffer
 * @param len Length of data to read
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_fast_read(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Get status register 1
 * 
 * @param status Pointer to store status
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_get_status1(uint8_t *status);

/**
 * @brief Get status register 2
 * 
 * @param status Pointer to store status
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_get_status2(uint8_t *status);

/**
 * @brief Get status register 3
 * 
 * @param status Pointer to store status
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_get_status3(uint8_t *status);

/**
 * @brief Set status register 1
 * 
 * @param status Status value to set
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_set_status1(uint8_t status);

/**
 * @brief Set status register 2
 * 
 * @param status Status value to set
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_set_status2(uint8_t status);

/**
 * @brief Set status register 3
 * 
 * @param status Status value to set
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_set_status3(uint8_t status);

/**
 * @brief Lock a specific block for write protection
 * 
 * @param addr Address within the block to lock
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_individual_block_lock(uint32_t addr);

/**
 * @brief Unlock a specific block
 * 
 * @param addr Address within the block to unlock
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_individual_block_unlock(uint32_t addr);

/**
 * @brief Read the lock status of a block
 * 
 * @param addr Address within the block to check
 * @param locked Pointer to store lock status (1 if locked, 0 if unlocked)
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_read_block_lock(uint32_t addr, uint8_t *locked);

/**
 * @brief Deinitialize the flash memory
 * 
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_advance_deinit(void);

/**
 * @brief      advance example get the jedec id information
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get jedec id failed
 * @note       none
 */
uint8_t w25qxx_advance_get_get_jedec_id(uint8_t *manufacturer, uint8_t device_id[2]);

/**
 * @brief  advance example global block lock
 * @return status code
 *         - 0 success
 *         - 1 global block lock failed
 * @note   none
 */
uint8_t w25qxx_advance_global_block_lock(void);

/**
 * @brief  advance example global block unlock
 * @return status code
 *         - 0 success
 *         - 1 global block unlock failed
 * @note   none
 */
uint8_t w25qxx_advance_global_block_unlock(void);

/**
 * @brief      advance example read only in the spi interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 only spi read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_read(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief      advance example fast read only in the spi dual output interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_fast_read_dual_output(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief      advance example fast read only in the spi quad output interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_fast_read_quad_output(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief      advance example fast read only in the spi dual io interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_fast_read_dual_io(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief      advance example fast read only in the spi quad io interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_fast_read_quad_io(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief      advance example word read only in the spi quad io interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_word_read_quad_io(uint32_t addr, uint8_t *data, uint32_t len);
 
/**
 * @brief      advance example octal word read only in the spi quad io interface
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_octal_word_read_quad_io(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief     advance example quad page program only in the spi quad input interface
 * @param[in] addr programming address
 * @param[in] *data pointer to a data buffer
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 quad page program failed
 * @note      len <= 256
 */
uint8_t w25qxx_advance_only_spi_page_program_quad_input(uint32_t addr, uint8_t *data, uint16_t len);

/**
 * @brief      advance example get the manufacturer && device id information only in the spi dual io interface
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get manufacturer device id dual io failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_get_manufacturer_device_id_dual_io(uint8_t *manufacturer, uint8_t *device_id);

/**
 * @brief      advance example get the manufacturer && device id information only in the spi quad io interface
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get manufacturer device id quad io failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_get_manufacturer_device_id_quad_io(uint8_t *manufacturer, uint8_t *device_id);

/**
 * @brief      advance example get the unique id only in the spi interface
 * @param[out] *id pointer to a id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get the unique id failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_get_unique_id(uint8_t id[8]);

/**
 * @brief      advance example get the sfdp only in the spi interface
 * @param[out] *sfdp pointer to a sfdp buffer
 * @return     status code
 *             - 0 success
 *             - 1 get the sfdp failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_get_sfdp(uint8_t sfdp[256]);

/**
 * @brief     advance example write the security register only in the spi interface
 * @param[in] num security register number
 * @param[in] *data pointer to a data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write security register failed
 * @note      none
 */
uint8_t w25qxx_advance_only_spi_write_security_register(w25qxx_security_register_t num, uint8_t data[256]);

/**
 * @brief      advance example read the security register only in the spi interface
 * @param[in]  num security register number
 * @param[out] *data pointer to a data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read security register failed
 * @note       none
 */
uint8_t w25qxx_advance_only_spi_read_security_register(w25qxx_security_register_t num, uint8_t data[256]);

/**
 * @brief     advance example set the burst with wrap only in the spi interface
 * @param[in] wrap burst wrap
 * @return    status code
 *            - 0 success
 *            - 1 set burst with wrap failed
 * @note      none
 */
uint8_t w25qxx_advance_only_spi_set_burst_with_wrap(w25qxx_burst_wrap_t wrap);

/**
 * @brief     advance example set the read parameters only in the qspi interface
 * @param[in] dummy qspi read dummy
 * @param[in] length qspi read wrap length
 * @return    status code
 *            - 0 success
 *            - 1 set read parameters failed
 * @note      none
 */
uint8_t w25qxx_advance_only_qspi_set_read_parameters(w25qxx_qspi_read_dummy_t dummy, w25qxx_qspi_read_wrap_length_t length);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
