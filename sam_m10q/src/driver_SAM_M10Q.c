#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_sam_m10q.h"

#define UBLOX_I2C_ADDRESS 0x42     /*!< Slave address of the UBlox device */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

static const char *TAG = "SAM-M10Q";

// Static variables for I2C configuration
static int sda_pin;
static int scl_pin;
static int i2c_port;
static uint32_t i2c_freq;

// FAST MODE Hex Codes
static const uint8_t fastModeHex1[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 
                                     0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x53, 0xCC};
static const uint8_t fastModeHex2[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 
                                     0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x52, 0xC3};

esp_err_t sam_m10q_init(int sda, int scl, int port, uint32_t freq) {
    sda_pin = sda;
    scl_pin = scl;
    i2c_port = port;
    i2c_freq = freq;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .master.clk_speed = i2c_freq,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t ret = i2c_param_config(i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed");
        return ret;
    }

    ret = i2c_driver_install(i2c_port, conf.mode, 
                           I2C_MASTER_RX_BUF_DISABLE, 
                           I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

static esp_err_t sendHexCode(const uint8_t *hexCodes, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (UBLOX_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, hexCodes, length, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmission error: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sam_m10q_set_10hz(void) {
    ESP_LOGI(TAG, "Changing CFG-RATE-MEAS to 100ms...");
    esp_err_t ret = sendHexCode(fastModeHex1, sizeof(fastModeHex1));
    if (ret != ESP_OK) return ret;
    
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

esp_err_t sam_m10q_read_nmea(void) {
    uint8_t data[255];
    char *fields[20];  // Array to store parsed fields
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (UBLOX_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data) - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    data[sizeof(data) - 1] = '\0';  // Null-terminate the string
    
    // Parse each line of NMEA data
    char *line = (char *)data;
    char *next_line;
    
    while ((next_line = strchr(line, '\n')) != NULL) {
        *next_line = '\0';  // Null-terminate this line
        
        // Only parse if it's a valid NMEA sentence (starts with $)
        if (line[0] == '$') {
            int num_fields = parse_comma_delimited_str(line, fields, 20);
            
            // Only print if it's an RMC, GGA, or VTG message
            if (strstr(fields[0], "RMC") != NULL || 
                strstr(fields[0], "GGA") != NULL || 
                strstr(fields[0], "VTG") != NULL) {
                printf("Message Type: %s\n", fields[0]);
                for (int i = 1; i <= num_fields; i++) {
                    printf("Field %d: %s\n", i, fields[i]);
                }
                printf("\n");
            }
        }
        
        line = next_line + 1;  // Move to the start of the next line
    }

    return ESP_OK;
}

void app_main() {
    ESP_ERROR_CHECK(sam_m10q_init(sda_pin, scl_pin, i2c_port, i2c_freq));

    ESP_ERROR_CHECK(sam_m10q_set_10hz());
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // let it cook for a lil lol

    while (1) {
        ESP_ERROR_CHECK(sam_m10q_read_nmea());
        vTaskDelay(333 / portTICK_PERIOD_MS);  // read at 3hz
    }
}