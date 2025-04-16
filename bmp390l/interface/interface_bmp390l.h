#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_BMP390L.h"
#include "i2c_manager.h"

uint32_t barometricAGL(void);
void set_ground_pressure(double pressure);