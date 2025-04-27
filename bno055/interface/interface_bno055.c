#include "interface_bno055.h"
#include "driver_bno055.h"

// Static calibration matrices and bias vectors
static float acc_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float gyr_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static float mag_correction_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

// Bias vectors initialized to zero
static float acc_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float gyr_bias_vector[3] = {0.0f, 0.0f, 0.0f};
static float mag_bias_vector[3] = {0.0f, 0.0f, 0.0f};

// Constant for rotation calculations
static const float SQRT2_2 = 0.70710678118f; // sqrt(2)/2 = cos(45°) = sin(45°)

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

void bno055_set_calibration(
    float acc_matrix[3][3], float gyr_matrix[3][3], float mag_matrix[3][3],
    float acc_bias[3], float gyr_bias[3], float mag_bias[3]
) {
    // Copy calibration matrices
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            acc_correction_matrix[i][j] = acc_matrix[i][j];
            gyr_correction_matrix[i][j] = gyr_matrix[i][j];
            mag_correction_matrix[i][j] = mag_matrix[i][j];
        }
    }
    
    // Copy bias vectors
    for (int i = 0; i < 3; i++) {
        acc_bias_vector[i] = acc_bias[i];
        gyr_bias_vector[i] = gyr_bias[i];
        mag_bias_vector[i] = mag_bias[i];
    }
}

esp_err_t bno055_get_raw(imu_raw_3d_t* acc, imu_raw_3d_t* gyr, imu_raw_3d_t* mag) {
    bno_readamg(
        &acc->x, &acc->y, &acc->z,
        &gyr->x, &gyr->y, &gyr->z,
        &mag->x, &mag->y, &mag->z
    );

    return ESP_OK;
}

esp_err_t bno055_get_calibrated(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out) {
    // Temporary structs for raw readings
    imu_raw_3d_t raw_acc, raw_gyr, raw_mag;
    
    // Get raw sensor data
    esp_err_t ret = bno055_get_raw(&raw_acc, &raw_gyr, &raw_mag);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Temporary arrays for floating point calculations
    float acc_float[3], gyr_float[3], mag_float[3];
    float acc_cal[3], gyr_cal[3], mag_cal[3];

    // Convert raw readings to float
    acc_float[0] = (float)raw_acc.x;
    acc_float[1] = (float)raw_acc.y;
    acc_float[2] = (float)raw_acc.z;

    gyr_float[0] = (float)raw_gyr.x;
    gyr_float[1] = (float)raw_gyr.y;
    gyr_float[2] = (float)raw_gyr.z;

    mag_float[0] = (float)raw_mag.x;
    mag_float[1] = (float)raw_mag.y;
    mag_float[2] = (float)raw_mag.z;

    // Apply calibrations
    apply_calibration(acc_float, acc_correction_matrix, acc_bias_vector, acc_cal);
    apply_calibration(gyr_float, gyr_correction_matrix, gyr_bias_vector, gyr_cal);
    apply_calibration(mag_float, mag_correction_matrix, mag_bias_vector, mag_cal);

    // Store calibrated results in output
    acc_out->x = (int16_t)acc_cal[0];
    acc_out->y = (int16_t)acc_cal[1];
    acc_out->z = (int16_t)acc_cal[2];

    gyr_out->x = (int16_t)gyr_cal[0];
    gyr_out->y = (int16_t)gyr_cal[1];
    gyr_out->z = (int16_t)gyr_cal[2];

    mag_out->x = (int16_t)mag_cal[0];
    mag_out->y = (int16_t)mag_cal[1];
    mag_out->z = (int16_t)mag_cal[2];

    return ESP_OK;
}

esp_err_t bno055_get_local(imu_raw_3d_t* acc_out, imu_raw_3d_t* gyr_out, imu_raw_3d_t* mag_out, bool local_up_flipped) {
    // First get calibrated readings
    esp_err_t ret = bno055_get_calibrated(acc_out, gyr_out, mag_out);
    if (ret != ESP_OK) {
        return ret;
    }

    // Temporary arrays for rotation calculations
    float acc_cal[3], gyr_cal[3], mag_cal[3];
    float acc_rot[3], gyr_rot[3], mag_rot[3];

    // Convert calibrated readings to float for rotation
    acc_cal[0] = (float)acc_out->x;
    acc_cal[1] = (float)acc_out->y;
    acc_cal[2] = (float)acc_out->z;

    gyr_cal[0] = (float)gyr_out->x;
    gyr_cal[1] = (float)gyr_out->y;
    gyr_cal[2] = (float)gyr_out->z;

    mag_cal[0] = (float)mag_out->x;
    mag_cal[1] = (float)mag_out->y;
    mag_cal[2] = (float)mag_out->z;

    // Apply -45° rotation
    rotate_z_45(acc_cal, acc_rot);
    rotate_z_45(gyr_cal, gyr_rot);
    rotate_z_45(mag_cal, mag_rot);

    // Flip X axis if needed
    flip_x_if_needed(acc_rot, local_up_flipped);
    flip_x_if_needed(gyr_rot, local_up_flipped);
    flip_x_if_needed(mag_rot, local_up_flipped);

    // Store rotated results
    acc_out->x = (int16_t)acc_rot[0];
    acc_out->y = (int16_t)acc_rot[1];
    acc_out->z = (int16_t)acc_rot[2];

    gyr_out->x = (int16_t)gyr_rot[0];
    gyr_out->y = (int16_t)gyr_rot[1];
    gyr_out->z = (int16_t)gyr_rot[2];

    mag_out->x = (int16_t)mag_rot[0];
    mag_out->y = (int16_t)mag_rot[1];
    mag_out->z = (int16_t)mag_rot[2];

    return ESP_OK;
}
