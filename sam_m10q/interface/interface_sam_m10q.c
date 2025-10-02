#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <inttypes.h>

#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"
#include "driver_SAM_M10Q.h"

#define GPS_RETRY_DELAY 50

void GPS_init(void) {
    esp_err_t ret;
    sam_m10q_msginfo_t msginfo;
    uint8_t gps_packet_buf[100]; // max buffer size needed for initialization. ubx messages can of course be larger than 100 bytes.
    uint16_t gps_packet_length; 

    disableNMEAMessages();
    int attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(GPS_RETRY_DELAY/portTICK_PERIOD_MS);
            attempts++;
        }
        if (msginfo.id != 0x01) {
            printf("Failed to disable NMEA messages, Retry # %d\n", attempts);
            for (int i = 0; i < gps_packet_length; i++) {
                printf("0x%02X ", gps_packet_buf[i]);
            }
            printf("\n");
        }
    } while (
        ret != ESP_OK && 
        attempts < 100 && 
        msginfo.id != 0x01 // UBX-ACK-ACK
    );

    setGPS10hz();
    attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(GPS_RETRY_DELAY/portTICK_PERIOD_MS);
            attempts++;
        }
        if (msginfo.id != 0x01) {
            printf("Failed to set GPS to 10hz, Retry # %d\n", attempts);
            for (int i = 0; i < gps_packet_length; i++) {
                printf("0x%02X ", gps_packet_buf[i]);
            }
            printf("\n");
        }
    } while (
        ret != ESP_OK &&
        attempts < 15 &&
        msginfo.id != 0x01 // UBX-ACK-ACK
    );
    
    setAirborneDynamicModel();
    attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(GPS_RETRY_DELAY/portTICK_PERIOD_MS);
            attempts++;
        }
        if (msginfo.id != 0x01) {
            printf("Failed to set GPS to Airborne Dynamic Model, Retry # %d\n", attempts);
            for (int i = 0; i < gps_packet_length; i++) {
                printf("0x%02X ", gps_packet_buf[i]);
            }
            printf("\n");
        }
    } while (
        ret != ESP_OK &&
        attempts < 15 &&
        msginfo.id != 0x01 // UBX-ACK-ACK
    );

    enableAllConstellations();
    attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(GPS_RETRY_DELAY/portTICK_PERIOD_MS);
            attempts++;
        }
        if (msginfo.id != 0x01) {
            printf("Failed to enable all constellations, Retry # %d\n", attempts);
            for (int i = 0; i < gps_packet_length; i++) {
                printf("0x%02X ", gps_packet_buf[i]);
            }
            printf("\n");
        }
    } while (
        ret != ESP_OK &&
        attempts < 15 &&
        msginfo.id != 0x01 // UBX-ACK-ACK
    );
    
}


void GPS_read(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *height, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV) {
    esp_err_t ret;
    sam_m10q_msginfo_t msginfo;
    uint8_t gps_packet_buf[GPS_MAX_PACKET_SIZE];
    uint16_t gps_packet_length;

    int attempts = 0;

    ret = reqNAVPVT(); // request NAV-PVT from the GPS
    if (ret != ESP_OK) {
        *UTCtstamp = 0;
        *lon = 0;
        *lat = 0;
        *hMSL = 0;
        *height = 0;
        *fixType = 0;
        *numSV = 0;
        printf("!!!!! WRITING TO GPS FAILED !!!!!!\n"); // todo: send the board into a fail state
    };

    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length); // read the response (i.e. next packet from the GPS)
        if (ret != ESP_OK) {
            vTaskDelay(GPS_RETRY_DELAY/portTICK_PERIOD_MS);  // arbritary retry delay
            attempts++;
        }
    } while (
        ret != ESP_OK &&
        attempts < 15 &&    // arbritary number
        msginfo.id != 0x07  // nav-pvt message ID
    );
    
    sam_m10q_navpvt_t navpvt = gpsParseNavPVT(); // now that we have a nav-pvt message, parse useful info from it

    // yeet the information at pointers
    // this is what the flight state logic and telemetry will use
    *UTCtstamp = navpvt.iTOW;
    *lon = navpvt.lon;
    *lat = navpvt.lat;
    *hMSL = navpvt.hMSL;
    *height = navpvt.height;
    *fixType = navpvt.fixType;
    *numSV = navpvt.numSV;
}
