#ifndef INTERFACE_BMP390L_H
#define INTERFACE_BMP390L_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "i2c_manager.h"
#include "driver_BMP390L.h"
#include "ascent_r2_hardware_definition.h"

typedef struct {
    double pressure;
    double temperature;
    double alt;
} baro_double_t;

extern SemaphoreHandle_t bmp390_mutex;

/**
 * @brief Initialize the BMP390 interface
 * 
 * This function initializes the mutex for protecting I2C bus access
 * during BMP390 sensor readings. It should be called once during system
 * initialization, before any sensor operations.
 */
void bmp390_interface_init(void);

/**
 * @brief Update ground pressure and temperature by averaging multiple readings
 * 
 * @param groundPressure Pointer to store the average ground pressure
 * @param groundTemperature Pointer to store the average ground temperature
 * @param num_readings Number of readings to average
 */
void update_ground_pressure(double *groundPressure, double *groundTemperature, uint8_t num_readings);

/**
 * @brief Convert pressure and temperature to altitude
 * 
 * @param pressure Pointer to pressure value in hPa
 * @param temperature Pointer to temperature value in Celsius
 * @param alt Pointer to store the calculated altitude in meters
 */
void pressure_to_m(double *pressure, double *temperature, double *alt);

/**
 * @brief Set calibration parameters for the BMP390 barometer
 * 
 * @param scaling Scaling factor for pressure measurements
 * @param bias Bias value for pressure measurements
 */
void bmp390_set_calibration(float scaling, float bias);

/**
 * @brief Read raw data from BMP390
 * 
 * @param baro_out Pointer to store barometer data
 */
void bmp390_get_raw(baro_double_t* baro_out);

/**
 * @brief Get calibrated readings from BMP390
 * 
 * @param baro_out Pointer to store calibrated barometer data
 */
void bmp390_get_calibrated(baro_double_t* baro_out);

/**
 * @brief Get altitude above ground level
 * 
 * @param baro_out Pointer to store altitude data in local reference frame
 */
void bmp390_get_local(baro_double_t* baro_out);

/**
 * @brief Store ground altitude for local reference frame
 * 
 * @param ground_alt Ground altitude in meters
 */
void bmp390_set_ground_alt(double ground_alt);

#endif /* INTERFACE_BMP390L_H */