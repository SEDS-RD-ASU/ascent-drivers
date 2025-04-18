#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "esp_err.h"
#include "ascent_r2_hardware_definition.h"

#define UBLOX_I2C_ADDRESS 0x42     /*!< Slave address of the UBlox device */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

static const char *TAG = "SAM-M10Q";

// FAST MODE Hex Codes
static const uint8_t fastModeHex1[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 
                                     0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x53, 0xCC};
static const uint8_t fastModeHex2[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 
                                     0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x52, 0xC3};

static esp_err_t sendHexCode(const uint8_t *hexCodes, size_t length) {
    esp_err_t ret = i2c_manager_write_register(I2C_MASTER_PORT, UBLOX_I2C_ADDRESS, 0x00, (uint8_t *)hexCodes, length);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmission error: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sam_m10q_set_10hz(void) {
    ESP_LOGI(TAG, "Changing CFG-RATE-MEAS to 100ms...");
    esp_err_t ret = sendHexCode(fastModeHex1, sizeof(fastModeHex1));
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100));  // Add a delay of 100 ms

    ret = sendHexCode(fastModeHex2, sizeof(fastModeHex2));
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Successfully set to 10Hz mode");
    return ESP_OK;
}

char* readNmeaStream() {
    size_t length = 500;
    char *data = (char *)malloc(length);
    if (!data) {
        printf("Failed to allocate memory for NMEA stream\n");
        return NULL;
    }

    esp_err_t ret = i2c_manager_read_register(I2C_MASTER_PORT, UBLOX_I2C_ADDRESS, 0xFF, (uint8_t *)data, length - 1);
    if (ret == ESP_ERR_TIMEOUT) {
        printf("Timeout occurred while reading NMEA stream\n");
        memset(data, 0, length);
        return data;
    } else if (ret != ESP_OK) {
        printf("Failed to read NMEA stream: %s\n", esp_err_to_name(ret));
        free(data);
        return NULL;
    }

    data[length - 1] = '\0';  // Ensure null-termination

    return data;
}