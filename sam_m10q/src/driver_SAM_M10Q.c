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

#include "driver_SAM_M10Q.h"

#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

uint16_t ubxAvailableBytes()
{   
    uint16_t avail = 0;
    uint8_t buf[2] = {0};
    i2c_manager_read_register(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, 0xFD, buf, 2);
    avail = ((uint16_t)buf[1] << 8) | buf[0];

    return avail;
}

