#include "interface_bmp390l.h"
#include "driver_BMP390L.h"
#include "math.h"
#include "driver_buzzer.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "ascent_r2_hardware_definition.h"
#include "freertos/semphr.h"

// Static calibration parameters
static float bmp_scaling = 1.0f;  // Default to no scaling
static float bmp_bias = 0.0f;     // Default to no bias
static double groundAlt = 0.0;    // Ground altitude for local reference

// Add the mutex definition
SemaphoreHandle_t bmp390_mutex = NULL;

// Initialize mutex in a new initialization function
void bmp390_interface_init(void) {
    if (bmp390_mutex == NULL) {
        bmp390_mutex = xSemaphoreCreateMutex();
    }
}

void update_ground_pressure(double *groundPressure, double *groundTemperature, uint8_t num_readings) {
    *groundPressure = 1013.25; // Default ground pressure in hPa
    *groundTemperature = 25.0; // Default ground temperature in Celsius

    double totalPressure = 0.0;    // Initialize total pressure
    double totalTemperature = 0.0; // Initialize total temperature

    // Loop over the number of readings
    for (int i = 0; i < num_readings; i++) {
        double pressure, temperature; // Declare pressure and temperature variables

        // Take mutex before reading sensor data, using the defined timeout constant
        if (xSemaphoreTake(bmp390_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT)) == pdTRUE) {
            // Read sensor data and store the return value
            esp_err_t ret = bmp390_read_sensor_data(&pressure, &temperature);
            xSemaphoreGive(bmp390_mutex);

            // If the sensor data read is not successful, log an error and return
            if (ret != ESP_OK) {
                ESP_LOGE("BMP390L", "Failed to read sensor data");
                return;
            }

            totalPressure += pressure;       // Add the pressure to the total pressure
            totalTemperature += temperature; // Add the temperature to the total temperature
        } else {
            ESP_LOGE("BMP390L", "Failed to get mutex for reading sensor data");
            // Skip this reading or handle timeout
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(30)); // Delay for 30 ms
    }

    // Calculate the average ground pressure and temperature
    *groundPressure = totalPressure / num_readings;  
    *groundTemperature = totalTemperature / num_readings;
}

void pressure_to_m(double *pressure, double *temperature, double *alt) {
    if (*pressure <= 0.0) {
        printf("Invalid pressure input: %.2f\n", *pressure);
        *alt = 0.0;
        return;
    }

    *alt = ((*temperature+273.15)/0.0065) * (1.0 - pow(*pressure / 1013.25, 1.0 / 5.255));
}

void bmp390_set_calibration(float scaling, float bias) {
    bmp_scaling = scaling;
    bmp_bias = bias;
}

void bmp390_set_ground_alt(double ground_alt) {
    groundAlt = ground_alt;
}

// Update the raw data read function to use the mutex with the defined timeout constant
void bmp390_get_raw(baro_double_t* baro_out) {
    if (xSemaphoreTake(bmp390_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT)) == pdTRUE) {
        bmp390_read_sensor_data(&baro_out->pressure, &baro_out->temperature);
        xSemaphoreGive(bmp390_mutex);
    } else {
        // Handle mutex timeout - set to default values or report error
        ESP_LOGE("BMP390L", "Failed to get mutex for reading sensor data");
        baro_out->pressure = 0;
        baro_out->temperature = 0;
    }
}

void bmp390_get_calibrated(baro_double_t* baro_out) {
    // Get raw data
    bmp390_get_raw(baro_out);
    
    // Apply scaling and bias correction
    baro_out->pressure = baro_out->pressure * bmp_scaling + bmp_bias;
    
    // Calculate altitude
    pressure_to_m(&baro_out->pressure, &baro_out->temperature, &baro_out->alt);
}

void bmp390_get_local(baro_double_t* baro_out) {
#ifdef FUNCTION_DURATION
    int64_t start_time = esp_timer_get_time(); // Get start time in microseconds
#endif

    // Get calibrated data
    bmp390_get_calibrated(baro_out);
    
    // Convert to altitude above ground level
    baro_out->alt = baro_out->alt - groundAlt;

#ifdef FUNCTION_DURATION
    int64_t end_time = esp_timer_get_time();
    float duration_ms = (end_time - start_time) / 1000.0;
    ESP_LOGI(BMP_TAG, "get_local execution time: %.3f ms", duration_ms);
#endif
}