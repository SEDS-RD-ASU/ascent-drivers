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


void GPS_Init(void) {
    esp_err_t ret;
    sam_m10q_msginfo_t msginfo;
    uint8_t gps_packet_buf[100];
    uint16_t gps_packet_length;

    disableNMEAMessages();
    int attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(10/portTICK_PERIOD_MS);
            attempts++;
        }
    } while (ret != ESP_OK && attempts < 15 && msginfo.id != 0x01);

    setGPS10hz();

    attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(10/portTICK_PERIOD_MS);
            attempts++;
        }
    } while (ret != ESP_OK && attempts < 15 && msginfo.id != 0x01);
}


void GPS_ReqNavPVT(uint32_t *UTCtstamp, double *lon, double *lat, int32_t *gps_altitude, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV) {
    esp_err_t ret;
    sam_m10q_msginfo_t msginfo;
    uint8_t gps_packet_buf[100];
    uint16_t gps_packet_length;

    int attempts = 0;
    reqNAVPVT();

    attempts = 0;
    do {
        ret = readNextGPSPacket(&msginfo, gps_packet_buf, &gps_packet_length);
        if (ret != ESP_OK) {
            vTaskDelay(10/portTICK_PERIOD_MS);
            attempts++;
        }
    } while (ret != ESP_OK && attempts < 15 && msginfo.id != 0x07);
    
    sam_m10q_navpvt_t navpvt = gpsParseNavPVT(gps_packet_buf);
    
    *UTCtstamp = navpvt.iTOW;
    *lon = navpvt.lon;
    *lat = navpvt.lat;
    *gps_altitude = navpvt.height;
    *hMSL = navpvt.hMSL;
    *fixType = navpvt.fixType;
    *numSV = navpvt.numSV;

    printf("UTC Timestamp: %" PRIu32 ", Longitude: %f, Latitude: %f, GPS Altitude: %" PRId32 ", HMSL: %" PRId32 ", Fix Type: %d, Number of Satellites: %d\n", *UTCtstamp, *lon, *lat, *gps_altitude, *hMSL, *fixType, *numSV);
}
