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

uint16_t ubxAvailableBytes()
{   
    uint16_t avail = 0;
    uint8_t buf[2] = {0};
    // read low byte from 0xFD, high byte from 0xFE
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFD, &buf[0], 1);
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFE, &buf[1], 1);
    avail = ((uint16_t)buf[1] << 8) | buf[0];

    return avail;
}

void ubxReadBytes(uint8_t *buf, uint16_t num_bytes)
{
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, buf, num_bytes);
}

void ubxDisableNMEA()
{
    uint8_t disable_nmea_msg[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x02, 0x00, 0x72, 0x10, 0x00, 0x1E, 0xB1
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, disable_nmea_msg, sizeof(disable_nmea_msg));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void ubxEnableNavPVT()
{
    uint8_t enable_navpvt_msg[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x06, 0x00, 0x91, 0x20, 0x01, 0x52, 0x43
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, enable_navpvt_msg, sizeof(enable_navpvt_msg));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void ubxResetGPS()
{
    uint8_t reset_msg[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x1F, 0x1F, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x00, 0x00, 0x07, 0x9F, 0xDE
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, reset_msg, sizeof(reset_msg));
}

void ubx10HzGPS()
{
    uint8_t set_10hz_msg[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x19, 0x00, 0x01, 0x00, 0x01, 0x00, 0x2F, 0x50
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, set_10hz_msg, sizeof(set_10hz_msg));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

// void ubx25hzGPS() // THIS IS PERMENANT. DO NOT USE ON AN ACTUAL ASCENT BOARD. THIS COULD BRICK THE GPS.
// {
//     printf("WARNING: You are about to set the GPS to 25Hz mode. THIS IS PERMANENT and should NOT be used on an actual ASCENT BOARD.\n");

//     uint8_t set_25hz_msg[] = {
//         0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E, 0x79, 0xBF, 0x28, 0xEF,
//         0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D, 0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00, 0x04, 0x01,
//         0xA4, 0x10, 0xBD, 0x34, 0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0,
//         0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE
//     };
//     i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, set_25hz_msg, sizeof(set_25hz_msg));
//     vTaskDelay(10/portTICK_PERIOD_MS);
// }

void ubxConstellations()
{
    uint8_t constellations_msg[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x4A, 0x00, 0x00, 0x03, 0x00, 0x00,
        0x1F, 0x00, 0x31, 0x10, 0x01, 0x01, 0x00, 0x31, 0x10, 0x01, 0x20,
        0x00, 0x31, 0x10, 0x00, 0x05, 0x00, 0x31, 0x10, 0x00, 0x21, 0x00,
        0x31, 0x10, 0x00, 0x07, 0x00, 0x31, 0x10, 0x00, 0x22, 0x00, 0x31,
        0x10, 0x00, 0x0D, 0x00, 0x31, 0x10, 0x00, 0x0F, 0x00, 0x31, 0x10,
        0x00, 0x24, 0x00, 0x31, 0x10, 0x00, 0x12, 0x00, 0x31, 0x10, 0x00,
        0x14, 0x00, 0x31, 0x10, 0x00, 0x25, 0x00, 0x31, 0x10, 0x00, 0x18,
        0x00, 0x31, 0x10, 0x00, 0x9F, 0xCF
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, constellations_msg, sizeof(constellations_msg));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void ubxReadStream(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *height, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV)
{
    uint16_t avail = ubxAvailableBytes();
    // printf("Available bytes in GPS stream: %u\n", avail);
    uint8_t byte = 0;
    int tries = 0;
    do {
        i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, &byte, 1);
        tries++;
    } while (byte != 0xB5);

    if (byte == 0xB5) {
        // printf("0xB5 found in stream after %d tries\n", tries);
    }
    if (tries > 500) {
        printf("Failed to find 0xB5 in stream after 500 tries\n");
        return;
    }

    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, &byte, 1);
    // printf("Second Byte: 0x%02X\n", byte);

    uint8_t header[4] = {0};
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, header, 4);

    uint8_t msg_class = header[0];
    uint8_t msg_id = header[1];
    uint16_t msg_length = ((uint16_t)header[3] << 8) | header[2];

    // printf("Message Class: 0x%02X, Message ID: 0x%02X, Length: %u\n\n", msg_class, msg_id, msg_length);

    if (msg_class == 0x01 && msg_id == 0x07 && msg_length == 92) {
        // Read the NAV-PVT payload (92 bytes)
        uint8_t navpvt_payload[92] = {0};
        i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, navpvt_payload, 92);

        // Parse some key fields from the payload
        *UTCtstamp = navpvt_payload[0] | (navpvt_payload[1] << 8) | (navpvt_payload[2] << 16) | (navpvt_payload[3] << 24);
        *lon = navpvt_payload[24] | (navpvt_payload[25] << 8) | (navpvt_payload[26] << 16) | (navpvt_payload[27] << 24);
        *lat = navpvt_payload[28] | (navpvt_payload[29] << 8) | (navpvt_payload[30] << 16) | (navpvt_payload[31] << 24);
        *height = navpvt_payload[32] | (navpvt_payload[33] << 8) | (navpvt_payload[34] << 16) | (navpvt_payload[35] << 24);
        *hMSL = navpvt_payload[36] | (navpvt_payload[37] << 8) | (navpvt_payload[38] << 16) | (navpvt_payload[39] << 24);
        *fixType = navpvt_payload[20];
        *numSV = navpvt_payload[23];
    }
}

void ubxMsgOutCfg()
{
    uint8_t msgout_navpvt_100hz[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x03, 0x00, 0x00,
        0x06, 0x00, 0x91, 0x20, 0x01, 0x54, 0x53
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, msgout_navpvt_100hz, sizeof(msgout_navpvt_100hz));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void ubxFreezeTimePulse()
{
    uint8_t timepulse_10mhz_msg[] = { // 10MHz 50/50 TIMEPULSE1
        0xB5,0x62,0x06,0x31,0x20,0x00,  // Header/Command/Size [UBX-CFG-TP5 (06 31)]
        0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x96,0x98,0x00,0x80,0x96,0x98,0x00,
        0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x6F,0x00,0x00,0x00,
        0x23,0x02 // Fletcher checksum, correct for preceeding frame
    };
    i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, timepulse_10mhz_msg, sizeof(timepulse_10mhz_msg));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void ubxReadStreamTiming() // do not use in flight
{
    uint16_t avail = ubxAvailableBytes();
    printf("Available bytes in GPS stream: %u\n", avail);
    uint8_t byte = 0;
    int tries = 0;
    int64_t start_time, end_time;
    
    do {
        start_time = esp_timer_get_time();
        i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, &byte, 1);
        end_time = esp_timer_get_time();
        printf("I2C read time: %.2f ms\n", (end_time - start_time) / 1000.0);
        tries++;
    } while (byte != 0xB5);

    if (byte == 0xB5) {
        printf("0xB5 found in stream after %d tries\n", tries);
    }
    if (tries > 500) {
        printf("Failed to find 0xB5 in stream after 500 tries\n");
        return;
    }

    start_time = esp_timer_get_time();
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, &byte, 1);
    end_time = esp_timer_get_time();
    printf("I2C read time: %.2f ms\n", (end_time - start_time) / 1000.0);
    printf("Second Byte: 0x%02X\n", byte);

    uint8_t header[4] = {0};
    start_time = esp_timer_get_time();
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, header, 4);
    end_time = esp_timer_get_time();
    printf("I2C read time: %.2f ms\n", (end_time - start_time) / 1000.0);

    uint8_t msg_class = header[0];
    uint8_t msg_id = header[1];
    uint16_t msg_length = ((uint16_t)header[3] << 8) | header[2];

    printf("Message Class: 0x%02X, Message ID: 0x%02X, Length: %u\n\n", msg_class, msg_id, msg_length);

    if (msg_class == 0x01 && msg_id == 0x07 && msg_length == 92) {
        // Read the NAV-PVT payload (92 bytes)
        uint8_t navpvt_payload[92] = {0};
        start_time = esp_timer_get_time();
        i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFF, navpvt_payload, 92);
        end_time = esp_timer_get_time();
        printf("I2C read time: %.2f ms\n", (end_time - start_time) / 1000.0);

        // Parse some key fields from the payload
        uint32_t iTOW = navpvt_payload[0] | (navpvt_payload[1] << 8) | (navpvt_payload[2] << 16) | (navpvt_payload[3] << 24);
        int32_t lon = navpvt_payload[24] | (navpvt_payload[25] << 8) | (navpvt_payload[26] << 16) | (navpvt_payload[27] << 24);
        int32_t lat = navpvt_payload[28] | (navpvt_payload[29] << 8) | (navpvt_payload[30] << 16) | (navpvt_payload[31] << 24);
        int32_t height = navpvt_payload[32] | (navpvt_payload[33] << 8) | (navpvt_payload[34] << 16) | (navpvt_payload[35] << 24);
        int32_t hMSL = navpvt_payload[36] | (navpvt_payload[37] << 8) | (navpvt_payload[38] << 16) | (navpvt_payload[39] << 24);
        uint8_t fixType = navpvt_payload[20];
        uint8_t numSV = navpvt_payload[23];

        printf("NAV-PVT:\n");
        printf("  iTOW: %lu ms\n", iTOW);
        printf("  Fix Type: %u\n", fixType);
        printf("  Num SV: %u\n", numSV);
        printf("  Lon: %.7f deg\n", lon / 1e7);
        printf("  Lat: %.7f deg\n", lat / 1e7);
        printf("  Height: %.3f m\n", height / 1000.0);
        printf("  Height MSL: %.3f m\n", hMSL / 1000.0);
    }
}
