#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver_BMP390L.h"
#include "i2c_manager.h"

uint32_t bmp390_barometricAGL(double pressure_hPa, double groundPressure_hPa);
void bmp390_set_ground_pressure(double pressure);