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

void GPS_init(void) {
    ubxResetGPS();

    printf("Reset GPS!\n\n");

    printf("Disabling NMEA...\n\n");

    ubxDisableNMEA();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    printf("Setting 10hz...\n\n");

    ubx10HzGPS();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    printf("Setting constellations to GPS only...\n\n");

    ubxConstellations();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    printf("Setting 20hz for timepulse...\n\n");

    ubxFreezeTimePulse();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    printf("Enabling NAV_PVT messages...\n\n");

    ubxEnableNavPVT();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);

    printf("Setting rate for NAV_PVT messages...\n\n");

    ubxMsgOutCfg();

    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

void GPS_read(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *gps_altitude, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV) {
    ubxReadStream(UTCtstamp, lon, lat, gps_altitude, hMSL, fixType, numSV);
}

void GPS_timing_debug(void) {
    ubxReadStreamTiming();
}

void GPS_reset(void) {
    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ubxResetGPS();
    ubxReadStream(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    printf("Reset GPS!\n\n");
}
