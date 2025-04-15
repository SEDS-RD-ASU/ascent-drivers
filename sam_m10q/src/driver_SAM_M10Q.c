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

static int parse_comma_delimited_str(char *string, char **fields, int max_fields) {
    int i = 0;
    fields[i++] = string;
    while ((i < max_fields) && NULL != (string = strchr(string, ','))) {
        *string = '\0';
        fields[i++] = ++string;
    }
    return --i;
}

uint8_t* readNmeaStream() {
    size_t length = 500;
    uint8_t *data = malloc(length);
    if (!data) {
        printf("Failed to allocate memory for NMEA stream\n");
        return NULL;
    }

    esp_err_t ret = i2c_manager_read_register(I2C_MASTER_PORT, UBLOX_I2C_ADDRESS, 0xFF, data, length - 1);
    if (ret != ESP_OK) {
        printf("Failed to read NMEA stream: %s\n", esp_err_to_name(ret));
        free(data);
        return NULL;
    }

    data[length - 1] = '\0';  // Ensure null-termination

    // Parse the NMEA string into fields
    char *fields[20]; // Adjust the number of fields as needed
    int field_count = parse_comma_delimited_str((char*)data, fields, 20);

    // Optionally log the parsed fields
    for (int i = 0; i < field_count; i++) {
        printf("Field %d: %s\n", i, fields[i]);
    }

    printf("NMEA Data: %s\n", (char*)data);
    return data;
}