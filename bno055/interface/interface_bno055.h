#ifndef INTERFACE_BNO055_H
#define INTERFACE_BNO055_H

#include <stdint.h>  // For int16_t
#include <stdbool.h> // For bool
#include "esp_err.h" // For esp_err_t
#include "esp_log.h" // For ESP_LOGE
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // For SemaphoreHandle_t
#include "ascent_r2_hardware_definition.h" // For MUTEX_TIMEOUT

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} imu_raw_3d_t;

// Add the mutex declaration before the function declarations
extern SemaphoreHandle_t bno055_mutex;

/**
 * @brief Set calibration matrices and bias vectors for BNO055 data
 * 
 * @param acc_matrix 3x3 correction matrix for accelerometer
 * @param gyr_matrix 3x3 correction matrix for gyroscope
 * @param mag_matrix 3x3 correction matrix for magnetometer
 * @param acc_bias Bias vector for accelerometer
 * @param gyr_bias Bias vector for gyroscope
 * @param mag_bias Bias vector for magnetometer
 */
void bno055_set_calibration(
    float acc_matrix[3][3], float gyr_matrix[3][3], float mag_matrix[3][3],
    float acc_bias[3], float gyr_bias[3], float mag_bias[3]
);

/**
 * @brief Read raw data from BNO055
 * 
 * @param acc Pointer to store accelerometer data (raw 16-bit)
 * @param gyr Pointer to store gyroscope data (raw 16-bit)
 * @param mag Pointer to store magnetometer data (raw 16-bit)
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t bno055_get_raw(imu_raw_3d_t* acc, imu_raw_3d_t* gyr, imu_raw_3d_t* mag);

/**
 * @brief Get calibrated readings from BNO055
 * 
 * @param acc_out Pointer to store calibrated accelerometer data
 * @param gyr_out Pointer to store calibrated gyroscope data
 * @param mag_out Pointer to store calibrated magnetometer data
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t bno055_get_calibrated(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out);

/**
 * @brief Get readings in local frame (after rotation and flip if needed)
 * 
 * @param acc_out Pointer to store local frame accelerometer data
 * @param gyr_out Pointer to store local frame gyroscope data
 * @param mag_out Pointer to store local frame magnetometer data
 * @param local_up_flipped Whether the local up direction is flipped
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
esp_err_t bno055_get_local(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out, bool local_up_flipped);

// Add initialization function
void bno055_interface_init(void);

#endif // INTERFACE_BNO055_H
