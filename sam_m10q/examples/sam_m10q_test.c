#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver_SAM_M10Q.h"

#include "ascent_r2_hardware_definition.h"

void GPS_test() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // let it cook for a lil lol
}