#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "i2c_manager.h"
#include "driver_BMP390L.h"

typedef struct {
    double velocity;
    double pressure;
    double altitude;
    double temperature;
    double acceleration; // New field for acceleration
} bmp390_vals_t;


void pressure_to_m(double *pressure, double *temperature, double *alt);
void update_ground_pressure(double *groundPressure, double *groundTemperature, uint8_t num_readings);
void bmp390_sensorinit(void);