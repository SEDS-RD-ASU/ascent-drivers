typedef struct {
    double x;
    double y;
    double z;
} imu_float_3d_t;

/**
 * @brief Set calibration matrix and bias vector for H3LIS331DL high-G accelerometer
 * 
 * @param correction_matrix 3x3 correction matrix
 * @param bias_vector Bias vector
 */
void h3lis331dl_set_calibration(float correction_matrix[3][3], float bias_vector[3]);

/**
 * @brief Read raw acceleration data from H3LIS331DL
 * 
 * @param acc Pointer to store acceleration in g
 */
void h3lis331dl_get_raw(imu_float_3d_t* acc);

/**
 * @brief Get calibrated acceleration data from H3LIS331DL
 * 
 * @param acc_out Pointer to store calibrated acceleration in g
 */
void h3lis331dl_get_calibrated(imu_float_3d_t* acc_out);

/**
 * @brief Get acceleration data in local frame (rotated and possibly flipped)
 * 
 * @param acc_out Pointer to store local frame acceleration in g
 * @param local_up_flipped Whether the local up direction is flipped
 */
void h3lis331dl_get_local(imu_float_3d_t* acc_out, bool local_up_flipped);