#ifndef DRIVER_SAM_M10Q_H
#define DRIVER_SAM_M10Q_H

#include "esp_err.h"

/**
 * @brief Initialize the SAM-M10Q GPS module
 * 
 * @param sda The GPIO pin number for SDA
 * @param scl The GPIO pin number for SCL
 * @param port The I2C port number
 * @param freq The I2C frequency in Hz
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sam_m10q_init(int sda, int scl, int port, uint32_t freq);

/**
 * @brief Configure GPS to output at 10Hz
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sam_m10q_set_10hz(void);

/**
 * @brief Read NMEA data from GPS
 * This function will parse and print RMC, GGA, and VTG messages
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sam_m10q_read_nmea(void);

#endif /* DRIVER_SAM_M10Q_H */ 