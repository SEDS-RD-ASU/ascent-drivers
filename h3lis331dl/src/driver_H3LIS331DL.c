#include <stdio.h>
#include <math.h>    // Add this for fabs()
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_h3lis331dl.h"
#include "i2c_manager.h"

static const char *TAG = "H3LIS331DL";

#define H3LIS331DL_I2C_ADDR 0x18    // H3LIS331DL I2C address

// Modify init function to just store port number
static i2c_port_t current_i2c_port;

// Static variables to store scale and endian settings
static h3lis331dl_scale_t current_scale = H3LIS331DL_SCALE_100G;
static h3lis331dl_endian_t current_endian = H3LIS331DL_BIG_ENDIAN;

// Add these static function declarations at the top of the file, after the includes
static esp_err_t h3lis331dl_write_reg(uint8_t reg_addr, uint8_t data);
static esp_err_t h3lis331dl_read_reg(uint8_t reg_addr, uint8_t *data, size_t len);
static uint8_t get_int_reg_addr(h3lis331dl_interrupt_t int_num, uint8_t base_reg);

// Helper function to convert raw values to g's based on scale
double convert_raw_to_g(int16_t raw_value, bool is_threshold) {
    double scale_factor;
    switch (current_scale) {
        case H3LIS331DL_SCALE_100G:
            scale_factor = 100.0 / 32768.0;  // ±100g range
            break;
        case H3LIS331DL_SCALE_200G:
            scale_factor = 200.0 / 32768.0;  // ±200g range
            break;
        case H3LIS331DL_SCALE_400G:
            scale_factor = 400.0 / 32768.0;  // ±400g range
            break;
        default:
            scale_factor = 100.0 / 32768.0;  // Default to ±100g range
            break;
    }

    // If it's a threshold, we need to convert it differently
    if (is_threshold) {
        return raw_value * scale_factor; // Return in g's for threshold
    } else {
        return raw_value * scale_factor; // Return in g's for raw value
    }
}

uint8_t convert_g_to_raw(double g_value, bool is_threshold) {
    double max_g;
    switch (current_scale) {
        case H3LIS331DL_SCALE_100G:
            max_g = 100.0;
            break;
        case H3LIS331DL_SCALE_200G:
            max_g = 200.0;
            break;
        case H3LIS331DL_SCALE_400G:
            max_g = 400.0;
            break;
        default:
            max_g = 100.0;  // Default to ±100g range
            break;
    }

    // Convert g value to the appropriate register value
    double abs_g = fabs(g_value);
    uint8_t value;

    if (is_threshold) {
        // Convert g value to 7-bit threshold (0-127)
        value = (uint8_t)((abs_g * 127.0) / max_g);
    } else {
        // Convert g value to 16-bit raw value
        value = (uint8_t)((abs_g * 32768.0) / max_g); // 16-bit conversion
    }

    // Ensure we don't exceed the respective bit limits
    if (is_threshold) {
        return value & 0x7F; // 7-bit threshold
    } else {
        return value; // 16-bit raw value
    }
}

esp_err_t h3lis331dl_init(i2c_port_t port) {
    // Store only the port number since I2C is initialized in main
    current_i2c_port = port;
    
    // Check if I2C is already initialized
    if (!i2c_manager_is_initialized(port)) {
        ESP_LOGE(TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Verify chip ID
    uint8_t chip_id;
    esp_err_t ret = h3lis331dl_get_chip_id(&chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    // Expected chip ID for H3LIS331DL is 0x32
    if (chip_id != 0x32) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%x (expected 0x32)", chip_id);
        return ESP_ERR_INVALID_VERSION;
    }

    ESP_LOGI(TAG, "H3LIS331DL initialized successfully, chip ID: 0x%x", chip_id);

    // Read initial scale and endian settings
    uint8_t reg_value;
    ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        current_scale = (h3lis331dl_scale_t)(reg_value & 0x30);
        current_endian = (h3lis331dl_endian_t)(reg_value & 0x40);
    }
    
    return ret;
}

// Update register write function to use i2c_manager
static esp_err_t h3lis331dl_write_reg(uint8_t reg_addr, uint8_t data)
{
    return i2c_manager_write_register(current_i2c_port, H3LIS331DL_I2C_ADDR, 
                                    reg_addr, &data, 1);
}

// Update register read function to use i2c_manager
static esp_err_t h3lis331dl_read_reg(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_manager_read_register(current_i2c_port, H3LIS331DL_I2C_ADDR,
                                   reg_addr, data, len);
}

void h3lis331dl_task(void *pvParameters)
{
    // Initialize H3LIS331DL
    esp_err_t ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG1, 0x37);  // Enable all axes, 50 Hz data rate
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    
    ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, 0x00);  // ±100g full scale
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Failed to write accelerometer data");
    }
    

    int16_t accel_data[3];
    uint8_t data[6];

    while (1) {
        // Read accelerometer data
        esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_OUT_X_L | 0x80, data, 6);  // 0x80 for auto-increment
        if (ret == ESP_OK) {
            accel_data[0] = (int16_t)((data[1] << 8) | data[0]);  // X-axis
            accel_data[1] = (int16_t)((data[3] << 8) | data[2]);  // Y-axis
            accel_data[2] = (int16_t)((data[5] << 8) | data[4]);  // Z-axis

            ESP_LOGI(TAG, "Accel: X=%d, Y=%d, Z=%d", accel_data[0], accel_data[1], accel_data[2]);
        } else {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for 100ms
    }
}

esp_err_t h3lis331dl_get_chip_id(uint8_t *chip_id)
{
    return h3lis331dl_read_reg(H3LIS331DL_WHO_AM_I, chip_id, 1);
}

esp_err_t h3lis331dl_get_axes_config(h3lis331dl_axes_config_t *axes_config)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        *axes_config = (h3lis331dl_axes_config_t)(reg_value & 0x07);  // Mask bits 0-2
    }
    return ret;
}

esp_err_t h3lis331dl_set_axes_config(h3lis331dl_axes_config_t axes_config)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x07;  // Clear bits 0-2
        reg_value |= (uint8_t)axes_config;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG1, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_datarate(h3lis331dl_datarate_t *datarate)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        *datarate = (h3lis331dl_datarate_t)(reg_value & 0x18);  // Mask bits 3-4
    }
    return ret;
}

esp_err_t h3lis331dl_set_datarate(h3lis331dl_datarate_t datarate)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x18;  // Clear bits 3-4
        reg_value |= (uint8_t)datarate;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG1, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_power_mode(h3lis331dl_power_mode_t *power_mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        *power_mode = (h3lis331dl_power_mode_t)(reg_value & 0xE0);  // Mask bits 5-7
    }
    return ret;
}

esp_err_t h3lis331dl_set_power_mode(h3lis331dl_power_mode_t power_mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG1, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0xE0;  // Clear bits 5-7
        reg_value |= (uint8_t)power_mode;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG1, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_hpcf(h3lis331dl_hpcf_t *hpcf)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        *hpcf = (h3lis331dl_hpcf_t)(reg_value & 0x03);  // Mask bits 0-1
    }
    return ret;
}

esp_err_t h3lis331dl_set_hpcf(h3lis331dl_hpcf_t hpcf)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x03;  // Clear bits 0-1
        reg_value |= (uint8_t)hpcf;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG2, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_hp_interrupt(h3lis331dl_hp_interrupt_t *hp_int)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        *hp_int = (h3lis331dl_hp_interrupt_t)(reg_value & 0x0C);  // Mask bits 2-3
    }
    return ret;
}

esp_err_t h3lis331dl_set_hp_interrupt(h3lis331dl_hp_interrupt_t hp_int)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x0C;  // Clear bits 2-3
        reg_value |= (uint8_t)hp_int;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG2, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_filter_mode(h3lis331dl_filter_t *filter)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        *filter = (h3lis331dl_filter_t)(reg_value & 0x10);  // Mask bit 4
    }
    return ret;
}

esp_err_t h3lis331dl_set_filter_mode(h3lis331dl_filter_t filter)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x10;  // Clear bit 4
        reg_value |= (uint8_t)filter;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG2, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_hp_mode(h3lis331dl_hpm_t *hp_mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        *hp_mode = (h3lis331dl_hpm_t)(reg_value & 0x60);  // Mask bits 5-6
    }
    return ret;
}

esp_err_t h3lis331dl_set_hp_mode(h3lis331dl_hpm_t hp_mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x60;  // Clear bits 5-6
        reg_value |= (uint8_t)hp_mode;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG2, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_reboot_memory(void)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG2, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value |= 0x80;  // Set bit 7 (BOOT)
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG2, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int1_config(h3lis331dl_int1_data_t *int_cfg)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *int_cfg = (h3lis331dl_int1_data_t)(reg_value & 0x03);  // Mask bits 0-1
    }
    return ret;
}

esp_err_t h3lis331dl_set_int1_config(h3lis331dl_int1_data_t int_cfg)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x03;  // Clear bits 0-1
        reg_value |= (uint8_t)int_cfg;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int1_latch(bool *latch)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *latch = (reg_value & 0x04) ? true : false;  // Check bit 2 (LIR1)
    }
    return ret;
}

esp_err_t h3lis331dl_set_int1_latch(bool latch)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        if (latch) {
            reg_value |= 0x04;  // Set bit 2 (LIR1)
        } else {
            reg_value &= ~0x04;  // Clear bit 2 (LIR1)
        }
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int2_config(h3lis331dl_int2_data_t *int_cfg)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *int_cfg = (h3lis331dl_int2_data_t)(reg_value & 0x18);  // Mask bits 3-4
    }
    return ret;
}

esp_err_t h3lis331dl_set_int2_config(h3lis331dl_int2_data_t int_cfg)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x18;  // Clear bits 3-4
        reg_value |= (uint8_t)int_cfg;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int2_latch(bool *latch)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *latch = (reg_value & 0x20) ? true : false;  // Check bit 5 (LIR2)
    }
    return ret;
}

esp_err_t h3lis331dl_set_int2_latch(bool latch)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        if (latch) {
            reg_value |= 0x20;  // Set bit 5 (LIR2)
        } else {
            reg_value &= ~0x20;  // Clear bit 5 (LIR2)
        }
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int_pin_mode(h3lis331dl_int_pin_mode_t *mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *mode = (h3lis331dl_int_pin_mode_t)(reg_value & 0x40);  // Mask bit 6 (PP_OD)
    }
    return ret;
}

esp_err_t h3lis331dl_set_int_pin_mode(h3lis331dl_int_pin_mode_t mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x40;  // Clear bit 6 (PP_OD)
        reg_value |= (uint8_t)mode;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_int_level(h3lis331dl_int_level_t *level)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        *level = (h3lis331dl_int_level_t)(reg_value & 0x80);  // Mask bit 7 (IHL)
    }
    return ret;
}

esp_err_t h3lis331dl_set_int_level(h3lis331dl_int_level_t level)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG3, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x80;  // Clear bit 7 (IHL)
        reg_value |= (uint8_t)level;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG3, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_scale(h3lis331dl_scale_t *scale)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        *scale = (h3lis331dl_scale_t)(reg_value & 0x30);  // Mask bits 4-5
    }
    return ret;
}

esp_err_t h3lis331dl_set_scale(h3lis331dl_scale_t scale)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x30;  // Clear bits 4-5
        reg_value |= (uint8_t)scale;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, reg_value);
        if (ret == ESP_OK) {
            current_scale = scale;  // Update static variable
        }
    }
    return ret;
}

esp_err_t h3lis331dl_get_spi_mode(h3lis331dl_spi_mode_t *mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        *mode = (h3lis331dl_spi_mode_t)(reg_value & 0x01);  // Mask bit 0
    }
    return ret;
}

esp_err_t h3lis331dl_set_spi_mode(h3lis331dl_spi_mode_t mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x01;  // Clear bit 0
        reg_value |= (uint8_t)mode;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_endian(h3lis331dl_endian_t *endian)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        *endian = (h3lis331dl_endian_t)(reg_value & 0x40);  // Mask bit 6
    }
    return ret;
}

esp_err_t h3lis331dl_set_endian(h3lis331dl_endian_t endian)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x40;  // Clear bit 6
        reg_value |= (uint8_t)endian;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, reg_value);
        if (ret == ESP_OK) {
            current_endian = endian;  // Update static variable
        }
    }
    return ret;
}

esp_err_t h3lis331dl_get_bdu(h3lis331dl_bdu_t *bdu)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        *bdu = (h3lis331dl_bdu_t)(reg_value & 0x80);  // Mask bit 7
    }
    return ret;
}

esp_err_t h3lis331dl_set_bdu(h3lis331dl_bdu_t bdu)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG4, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x80;  // Clear bit 7
        reg_value |= (uint8_t)bdu;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG4, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_get_sleep_to_wake(h3lis331dl_sleep_to_wake_t *mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG5, &reg_value, 1);
    if (ret == ESP_OK) {
        // Only 0x00 and 0x03 are valid values according to the description
        uint8_t masked_value = reg_value & 0x03;  // Mask bits 0-1
        if (masked_value == 0x03) {
            *mode = H3LIS331DL_SLEEP_TO_WAKE_LOW_POWER;
        } else {
            *mode = H3LIS331DL_SLEEP_TO_WAKE_DISABLED;
        }
    }
    return ret;
}

esp_err_t h3lis331dl_set_sleep_to_wake(h3lis331dl_sleep_to_wake_t mode)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_CTRL_REG5, &reg_value, 1);
    if (ret == ESP_OK) {
        reg_value &= ~0x03;  // Clear bits 0-1
        reg_value |= (uint8_t)mode;
        ret = h3lis331dl_write_reg(H3LIS331DL_CTRL_REG5, reg_value);
    }
    return ret;
}

esp_err_t h3lis331dl_hp_filter_reset(void)
{
    uint8_t dummy_data;
    // Simply reading this register resets the high-pass filter
    return h3lis331dl_read_reg(H3LIS331DL_HP_FILTER_RESET, &dummy_data, 1);
}

esp_err_t h3lis331dl_set_reference(uint8_t reference)
{
    // First, write the reference value to the register
    esp_err_t ret = h3lis331dl_write_reg(H3LIS331DL_REFERENCE, reference);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Then, reset the high-pass filter to apply the reference value
    return h3lis331dl_hp_filter_reset();
}

esp_err_t h3lis331dl_get_reference(uint8_t *reference)
{
    return h3lis331dl_read_reg(H3LIS331DL_REFERENCE, reference, 1);
}

esp_err_t h3lis331dl_get_status(h3lis331dl_status_t *status)
{
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_STATUS_REG, &reg_value, 1);
    if (ret == ESP_OK) {
        // Parse individual bits into the status structure
        status->x_data_available = (reg_value & 0x01) != 0;    // Bit 0
        status->y_data_available = (reg_value & 0x02) != 0;    // Bit 1
        status->z_data_available = (reg_value & 0x04) != 0;    // Bit 2
        status->xyz_data_available = (reg_value & 0x08) != 0;  // Bit 3
        status->x_overrun = (reg_value & 0x10) != 0;          // Bit 4
        status->y_overrun = (reg_value & 0x20) != 0;          // Bit 5
        status->z_overrun = (reg_value & 0x40) != 0;          // Bit 6
        status->xyz_overrun = (reg_value & 0x80) != 0;        // Bit 7
    }
    return ret;
}

esp_err_t h3lis331dl_read_accel(double *x_accel, double *y_accel, double *z_accel) {
    uint8_t data[6];
    int16_t raw_values[3];

    // Read all acceleration registers in one transaction (auto-increment)
    esp_err_t ret = h3lis331dl_read_reg(H3LIS331DL_OUT_X_L | 0x80, data, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    // Combine bytes according to endianness
    if (current_endian == H3LIS331DL_BIG_ENDIAN) {
        raw_values[0] = (int16_t)((data[1] << 8) | data[0]);  // X-axis
        raw_values[1] = (int16_t)((data[3] << 8) | data[2]);  // Y-axis
        raw_values[2] = (int16_t)((data[5] << 8) | data[4]);  // Z-axis
    } else {
        raw_values[0] = (int16_t)((data[0] << 8) | data[1]);  // X-axis
        raw_values[1] = (int16_t)((data[2] << 8) | data[3]);  // Y-axis
        raw_values[2] = (int16_t)((data[4] << 8) | data[5]);  // Z-axis
    }

    // Convert raw values to g's
    *x_accel = convert_raw_to_g(raw_values[0], false);
    *y_accel = convert_raw_to_g(raw_values[1], false);
    *z_accel = convert_raw_to_g(raw_values[2], false);

    return ESP_OK;
}

// Helper function to get the appropriate register address based on interrupt number
static uint8_t get_int_reg_addr(h3lis331dl_interrupt_t int_num, uint8_t base_reg) {
    return base_reg + (int_num * 4);  // INT2 registers are 4 addresses higher than INT1
}

esp_err_t h3lis331dl_get_int_cfg(h3lis331dl_interrupt_t int_num, h3lis331dl_int1_config_t *config) {
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_CFG);
    uint8_t reg_value;
    esp_err_t ret = h3lis331dl_read_reg(reg_addr, &reg_value, 1);
    
    if (ret == ESP_OK) {
        config->x_low_enable = (reg_value & 0x01) != 0;    // XLIE
        config->x_high_enable = (reg_value & 0x02) != 0;   // XHIE
        config->y_low_enable = (reg_value & 0x04) != 0;    // YLIE
        config->y_high_enable = (reg_value & 0x08) != 0;   // YHIE
        config->z_low_enable = (reg_value & 0x10) != 0;    // ZLIE
        config->z_high_enable = (reg_value & 0x20) != 0;   // ZHIE
        config->interrupt_mode = (h3lis331dl_int_mode_t)(reg_value & 0x80); // AOI
    }
    return ret;
}

esp_err_t h3lis331dl_set_int_cfg(h3lis331dl_interrupt_t int_num, uint8_t enables_mask, bool and_mode)
{
    uint8_t reg_value = enables_mask & 0x3F;  // Mask to ensure only bits 0-5 are used
    if (and_mode) {
        reg_value |= 0x80;  // Set AOI bit for AND mode
    }
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_CFG);
    return h3lis331dl_write_reg(reg_addr, reg_value);
}

esp_err_t h3lis331dl_set_int_simple(h3lis331dl_interrupt_t int_num, h3lis331dl_axis_t axis, 
                                   h3lis331dl_int_event_t event_type, bool enable)
{
    if (axis > H3LIS331DL_AXIS_Z) {  // Validate axis parameter
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t current_value;
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_CFG);
    esp_err_t ret = h3lis331dl_read_reg(reg_addr, &current_value, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // Calculate bit position (0,1 for X, 2,3 for Y, 4,5 for Z)
    uint8_t bit_pos = ((uint8_t)axis * 2) + (uint8_t)event_type;
    uint8_t bit_mask = 1 << bit_pos;

    if (enable) {
        current_value |= bit_mask;   // Set bit
    } else {
        current_value &= ~bit_mask;  // Clear bit
    }

    // Preserve AOI bit (bit 7)
    return h3lis331dl_write_reg(reg_addr, current_value);
}

esp_err_t h3lis331dl_get_int_src(h3lis331dl_interrupt_t int_num, h3lis331dl_int1_src_t *src)
{
    uint8_t reg_value;
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_SRC);
    esp_err_t ret = h3lis331dl_read_reg(reg_addr, &reg_value, 1);
    
    if (ret == ESP_OK) {
        src->x_low = (reg_value & 0x01) != 0;           // Bit 0
        src->x_high = (reg_value & 0x02) != 0;          // Bit 1
        src->y_low = (reg_value & 0x04) != 0;           // Bit 2
        src->y_high = (reg_value & 0x08) != 0;          // Bit 3
        src->z_low = (reg_value & 0x10) != 0;           // Bit 4
        src->z_high = (reg_value & 0x20) != 0;          // Bit 5
        src->interrupt_active = (reg_value & 0x40) != 0; // Bit 6
        // Bit 7 is reserved
    }
    
    return ret;
}

esp_err_t h3lis331dl_set_int_threshold(h3lis331dl_interrupt_t int_num, double threshold_g) {
    // Check if threshold exceeds current scale
    double max_g;
    switch (current_scale) {
        case H3LIS331DL_SCALE_100G:
            max_g = 100.0;
            break;
        case H3LIS331DL_SCALE_200G:
            max_g = 200.0;
            break;
        case H3LIS331DL_SCALE_400G:
            max_g = 400.0;
            break;
        default:
            max_g = 100.0;
            break;
    }
    
    if (fabs(threshold_g) > max_g) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t threshold = convert_g_to_raw(threshold_g, true);
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_THS);
    return h3lis331dl_write_reg(reg_addr, threshold);
}

esp_err_t h3lis331dl_get_int_threshold(h3lis331dl_interrupt_t int_num, double *threshold_g) {
    uint8_t threshold;
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_THS);
    esp_err_t ret = h3lis331dl_read_reg(reg_addr, &threshold, 1);
    if (ret == ESP_OK) {
        *threshold_g = convert_raw_to_g(threshold, true);
    }
    return ret;
}

esp_err_t h3lis331dl_set_int_duration(h3lis331dl_interrupt_t int_num, uint8_t duration) {
    // Check if the duration is within valid limits based on ODR
    // Assuming ODR is set to 50Hz, the maximum duration can be calculated
    // For example, if ODR is 50Hz, the duration can be set from 0 to 127
    // (0-127 corresponds to 0 to 127 * 20ms = 0 to 2.54 seconds)
    
    // Here, we can define the maximum duration based on the ODR
    // For simplicity, let's assume the maximum is 127 for now
    // You can adjust this based on your ODR settings
    if (duration > 127) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_DURATION);
    return h3lis331dl_write_reg(reg_addr, duration);
}

esp_err_t h3lis331dl_get_int_duration(h3lis331dl_interrupt_t int_num, uint8_t *duration) {
    uint8_t reg_addr = get_int_reg_addr(int_num, H3LIS331DL_INT1_DURATION);
    return h3lis331dl_read_reg(reg_addr, duration, 1);
}