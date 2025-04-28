#include "interface_h3lis331dl.h"
#include "driver_H3LIS331DL.h"
#include <stdbool.h>
#include "freertos/semphr.h"
#include "ascent_r2_hardware_definition.h"  // Make sure this is included

// Static calibration matrix and bias vector
static float high_g_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float high_g_bias_vector[3] = {0.0f, 0.0f, 0.0f};

// Constant for rotation calculations
static const float SQRT2_2 = 0.70710678118f; // sqrt(2)/2 = cos(45°) = sin(45°)

// Add the mutex definition
SemaphoreHandle_t h3lis331dl_mutex = NULL;

// Initialize mutex in a new initialization function
void h3lis331dl_interface_init(void) {
    if (h3lis331dl_mutex == NULL) {
        h3lis331dl_mutex = xSemaphoreCreateMutex();
    }
}

// Helper function to apply 3x3 matrix multiplication and bias addition
static void apply_calibration(float* input, float matrix[3][3], float* bias, float* output) {
    // Matrix multiplication
    output[0] = matrix[0][0] * input[0] + matrix[0][1] * input[1] + matrix[0][2] * input[2];
    output[1] = matrix[1][0] * input[0] + matrix[1][1] * input[1] + matrix[1][2] * input[2];
    output[2] = matrix[2][0] * input[0] + matrix[2][1] * input[1] + matrix[2][2] * input[2];

    // Add bias
    output[0] += bias[0];
    output[1] += bias[1];
    output[2] += bias[2];
}

// Helper function to rotate a vector -45 degrees around Z axis
static void rotate_z_45(float* input, float* output) {
    float x = input[0];
    float y = input[1];
    
    // Rotation matrix multiplication for Z axis (-45 degrees)
    output[0] = SQRT2_2 * x + SQRT2_2 * y;   // x' = cos(-45°)x - sin(-45°)y
    output[1] = -SQRT2_2 * x + SQRT2_2 * y;  // y' = sin(-45°)x + cos(-45°)y
    output[2] = input[2];                     // z' = z (unchanged)
}

// Helper function to flip X axis if needed
static void flip_x_if_needed(float* vec, bool flip) {
    if (flip) {
        vec[0] = -vec[0];  // Negate X component
    }
}

void h3lis331dl_set_calibration(float correction_matrix[3][3], float bias_vector[3]) {
    // Copy calibration matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            high_g_correction_matrix[i][j] = correction_matrix[i][j];
        }
    }
    
    // Copy bias vector
    for (int i = 0; i < 3; i++) {
        high_g_bias_vector[i] = bias_vector[i];
    }
}

// Update the raw data read function to use the mutex with the defined timeout constant
void h3lis331dl_get_raw(imu_float_3d_t* acc) {
    if (acc) {
        if (xSemaphoreTake(h3lis331dl_mutex, pdMS_TO_TICKS(MUTEX_TIMEOUT)) == pdTRUE) {
            h3lis331dl_read_accel(&acc->x, &acc->y, &acc->z);
            xSemaphoreGive(h3lis331dl_mutex);
        } else {
            // Handle mutex timeout - set to default values or report error
            ESP_LOGE("H3LIS331DL", "Failed to get mutex for reading sensor data");
            acc->x = 0;
            acc->y = 0;
            acc->z = 0;
        }
    }
}

void h3lis331dl_get_calibrated(imu_float_3d_t* acc_out) {
    // Temporary struct for raw readings
    imu_float_3d_t raw_acc;
    
    // Get raw sensor data
    h3lis331dl_get_raw(&raw_acc);
    
    float acc_float[3], acc_cal[3];

    // Copy raw readings to array
    acc_float[0] = (float)raw_acc.x;
    acc_float[1] = (float)raw_acc.y;
    acc_float[2] = (float)raw_acc.z;

    // Apply calibration
    apply_calibration(acc_float, high_g_correction_matrix, high_g_bias_vector, acc_cal);

    // Store calibrated results
    acc_out->x = (double)acc_cal[0];
    acc_out->y = (double)acc_cal[1];
    acc_out->z = (double)acc_cal[2];
}

void h3lis331dl_get_local(imu_float_3d_t* acc_out, bool local_up_flipped) {
    // First get calibrated readings
    h3lis331dl_get_calibrated(acc_out);

    // Temporary arrays for rotation calculations
    float acc_cal[3], acc_rot[3];

    // Copy calibrated readings to array
    acc_cal[0] = (float)acc_out->x;
    acc_cal[1] = (float)acc_out->y;
    acc_cal[2] = (float)acc_out->z;

    // Apply -45° rotation
    rotate_z_45(acc_cal, acc_rot);

    // Flip X axis if needed
    flip_x_if_needed(acc_rot, local_up_flipped);

    // Store rotated results
    acc_out->x = (double)acc_rot[0];
    acc_out->y = (double)acc_rot[1];
    acc_out->z = (double)acc_rot[2];
}
