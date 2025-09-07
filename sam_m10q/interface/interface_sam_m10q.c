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


void GPS_read(uint32_t *UTCtstamp, int32_t *lon, int32_t *lat, int32_t *gps_altitude, int32_t *hMSL, uint8_t *fixType, uint8_t *numSV) {
    
}