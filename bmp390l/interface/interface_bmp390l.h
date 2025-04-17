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


float bmp390_barometricAGL(double pressure_hPa, double groundPressure_hPa);
void bmp390_set_ground_pressure(double pressure);

// Function to set the ground pressure and calculate average pressure
void update_ground_pressure();

// Function to calculate vertical velocity based on altitude samples
double calculate_velocity();

// Function to add a new altitude sample to the buffer
void add_sample(double altitude, int64_t timestamp_us);

// Function to update sensor values and calculate altitude and velocity
void update_vals(void *pvParameters);

// Function to retrieve current sensor values
bmp390_vals_t bmp390_vals(); 

void check_conditions_task(void *pvParameters);