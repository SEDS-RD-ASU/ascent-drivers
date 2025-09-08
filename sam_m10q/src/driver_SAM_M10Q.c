#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <string.h>
#include "esp_timer.h"

#include "driver_SAM_M10Q.h"

#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

#define GPS_MAX_PACKET_SIZE 1024

static uint8_t gps_packet_buf[GPS_MAX_PACKET_SIZE];


// ----------- GPS SPECIFIC I2C ------------------ //

static esp_err_t ubx_read_len(uint16_t *len) {
    uint8_t buf[2];
    uint8_t reg = 0xFD;

    // Write register address
    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, &reg, 1, buf, 2, pdMS_TO_TICKS(100));
    if (err != ESP_OK) return err;

    *len = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

static esp_err_t ubx_read_data(uint8_t *data, size_t len) {
    uint8_t reg = 0xFF;
    return i2c_master_write_read_device(
        I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t read_gps_stream(uint8_t *data, uint16_t buf_length, uint16_t *real_length) {
    ubx_read_len(real_length);
    if(*real_length == 0) {
        return ESP_FAIL;
    }
    if(*real_length > buf_length) {
        printf("ya fucked up. buf: %d, real: %d.\n", buf_length, *real_length);
        return ESP_FAIL;
    }
    ubx_read_data(data,*real_length);
    return ESP_OK;
}

// ------ actual driver follows ----  //

// This is the third full overhaul of this driver. I am really bad at writing drivers. This GPS is a pain in my ass. - Abdul

esp_err_t sendGPSBytes(uint8_t *buf, uint16_t num_bytes) {
    return i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, buf, num_bytes);
}


esp_err_t readGPSBytes(uint8_t *buf, uint16_t num_bytes) {
    return i2c_manager_read_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, buf, num_bytes);
}


esp_err_t readNextGPSPacket(void) {
    uint16_t packet_length = 0;

    esp_err_t ret = ESP_FAIL;
    for (int i = 0; i < 3; i++) {
        ret = read_gps_stream(gps_packet_buf, GPS_MAX_PACKET_SIZE, &packet_length);
        if (ret == ESP_OK) {
            break;
        }
    }

    if(ret == ESP_FAIL) return ESP_FAIL;

    sam_m10q_msginfo_t msginfo = sam_m10q_get_msginfo(gps_packet_buf, packet_length);

    printf("msginfo: class: 0x%02X, id: 0x%02X, length: %u\n", msginfo.class, msginfo.id, msginfo.length);

    printf("packet length: %d\n", packet_length);

    for(int i = 0; i < packet_length; i++){
        printf("0x%02X ", gps_packet_buf[i]);
    }

    printf("\n");

    return ESP_OK;
}

esp_err_t disableNMEAMessages(void) {
    uint8_t disable_nmea_msg[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x02, 0x00, 0x72, 0x10, 0x00, 0x1E, 0xB1
    };
    return sendGPSBytes(disable_nmea_msg, sizeof(disable_nmea_msg));
}

esp_err_t setGPS10hz(void)
{
    uint8_t set_10hz_msg[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x19, 0x00, 0x01, 0x00, 0x01, 0x00, 0x2F, 0x50
    };
    return sendGPSBytes(set_10hz_msg, sizeof(set_10hz_msg));
}


sam_m10q_msginfo_t sam_m10q_get_msginfo(uint8_t *buf, uint16_t bufsize) {
    sam_m10q_msginfo_t msginfo;
    msginfo.class = buf[2];
    msginfo.id = buf[3];
    msginfo.length = (buf[5] << 8) | (buf[4]);
    msginfo.valid_checksum = false;

    // Validate checksum

    uint8_t ck_a;
    uint8_t ck_b;

    ck_a = buf[bufsize - 2];
    ck_b = buf[bufsize - 1];

    return msginfo;
}
