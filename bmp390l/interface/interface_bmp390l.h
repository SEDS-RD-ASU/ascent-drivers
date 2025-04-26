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


float bmp390_barometricAGL();
void update_ground_pressure(double *groundPressure);
void bmp390_sensorinit(void);