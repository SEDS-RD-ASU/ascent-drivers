#include "i2c_manager.h"
#include <stdbool.h>

// Static array to track initialization status of I2C ports
static bool i2c_initialized[I2C_NUM_MAX] = {false};

esp_err_t i2c_manager_init(int sda_pin, int scl_pin, uint32_t freq_hz, i2c_port_t port) {
    // Check if already initialized
    if (i2c_initialized[port]) {
        return ESP_OK; // Already initialized, just return success
    }

    // Configure I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
    };

    esp_err_t ret = i2c_param_config(port, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    i2c_initialized[port] = true;
    return ESP_OK;
}

bool i2c_manager_is_initialized(i2c_port_t port) {
    if (port >= I2C_NUM_MAX) {
        return false;
    }
    return i2c_initialized[port];
}

esp_err_t i2c_manager_deinit(i2c_port_t port) {
    if (!i2c_initialized[port]) {
        return ESP_OK; // Already deinitialized
    }

    esp_err_t ret = i2c_driver_delete(port);
    if (ret == ESP_OK) {
        i2c_initialized[port] = false;
    }
    return ret;
}

esp_err_t i2c_manager_read_register(i2c_port_t port, uint8_t device_addr, 
                                  uint8_t reg, uint8_t *data, size_t len) {
    if (!i2c_initialized[port]) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Write the register address we want to read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Perform repeated start and read the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

esp_err_t i2c_manager_write_register(i2c_port_t port, uint8_t device_addr, 
                                   uint8_t reg, uint8_t *data, size_t len) {
    if (!i2c_initialized[port]) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
} 