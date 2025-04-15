/**
 * @file driver_BMP390L.h
 * @brief BMP390 Pressure and Temperature Sensor Driver
 *
 * This driver provides an interface for the Bosch BMP390 pressure and temperature sensor.
 * The BMP390 is a high-precision, low-power digital sensor capable of measuring both
 * barometric pressure and temperature.
 *
 * Features:
 * - Pressure measurement range: 300 hPa to 1250 hPa
 * - Temperature measurement range: -40°C to +85°C
 * - I2C interface support
 * - Configurable oversampling, IIR filtering, and output data rates
 * - Multiple operation modes: sleep, forced, and normal
 */

#ifndef DRIVER_BMP390L_H
#define DRIVER_BMP390L_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

// Type definitions
typedef struct {
    uint64_t pressure;     // Raw 24-bit pressure reading
    int64_t  temperature;  // Raw 24-bit temperature reading
} bmp3_uncomp_data_t;

typedef struct {
    double pressure;       // Compensated pressure in Pascals (Pa)
    double temperature;    // Compensated temperature in Celsius (°C)
} bmp3_data_t;

typedef struct {
    uint16_t par_t1;      // Temperature compensation coefficient 1
    int16_t  par_t2;      // Temperature compensation coefficient 2
    int8_t   par_t3;      // Temperature compensation coefficient 3
    uint16_t par_p1;
    int16_t  par_p2;
    int8_t   par_p3;
    int8_t   par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t   par_p7;
    int8_t   par_p8;
    int16_t  par_p9;
    int16_t  par_p10;
    int8_t   par_p11;
    double   t_lin;       // Intermediate temperature calculation value
} bmp390_calib_data_t;

// Enums for sensor configuration
typedef enum {
    BMP390_MODE_SLEEP = 0x00,   // Low power sleep mode
    BMP390_MODE_FORCED = 0x01,  // Single measurement mode
    BMP390_MODE_NORMAL = 0x03   // Continuous measurement mode
} bmp390_mode_t;

typedef enum {
    BMP390_OVERSAMPLING_1X = 0,    // No oversampling
    BMP390_OVERSAMPLING_2X = 1,    // 2x oversampling
    BMP390_OVERSAMPLING_4X = 2,    // 4x oversampling
    BMP390_OVERSAMPLING_8X = 3,    // 8x oversampling
    BMP390_OVERSAMPLING_16X = 4,   // 16x oversampling
    BMP390_OVERSAMPLING_32X = 5    // 32x oversampling
} bmp390_osr_t;

typedef enum {
    BMP390_ODR_200HZ = 0,      // 200 Hz sampling
    BMP390_ODR_100HZ = 1,      // 100 Hz sampling
    BMP390_ODR_50HZ  = 2,
    BMP390_ODR_25HZ  = 3,
    BMP390_ODR_12_5HZ = 4,
    BMP390_ODR_6_25HZ = 5,
    BMP390_ODR_3_125HZ = 6,
    BMP390_ODR_1_5625HZ = 7,
    BMP390_ODR_0_78125HZ = 8   // 0.78125 Hz sampling
} bmp390_odr_t;

typedef enum {
    BMP390_IIR_FILTER_BYPASS = 0,    // No filtering
    BMP390_IIR_FILTER_COEFF_1 = 1,   // Coefficient of 1
    BMP390_IIR_FILTER_COEFF_3 = 2,
    BMP390_IIR_FILTER_COEFF_7 = 3,
    BMP390_IIR_FILTER_COEFF_15 = 4,
    BMP390_IIR_FILTER_COEFF_31 = 5,
    BMP390_IIR_FILTER_COEFF_63 = 6,
    BMP390_IIR_FILTER_COEFF_127 = 7  // Coefficient of 127
} bmp390_iir_filter_t;

// Configuration structures
typedef struct {
    bool press_en;         // Pressure sensor enable flag
    bool temp_en;          // Temperature sensor enable flag
    bmp390_mode_t mode;    // Sensor operation mode
} bmp390_pwr_ctrl_t;

typedef struct {
    bmp390_osr_t press_os;
    bmp390_osr_t temp_os;
} bmp390_osr_settings_t;

typedef struct {
    bmp390_iir_filter_t iir_filter;
} bmp390_config_t;

typedef struct {
    bool cmd_rdy;
} bmp390_status_t;

// Constants
#define BMP3_OK           0
#define BMP3_PRESS_TEMP   3
#define BMP3_PRESS        1
#define BMP3_TEMP         2

// After the other register definitions, add:
#define BMP390_REG_INT_CTRL  0x19  // Interrupt control register

/**
 * @brief Interrupt configuration structure
 * 
 * Controls the behavior of the BMP390's interrupt pin and various
 * interrupt sources:
 * - Data Ready interrupt
 * - FIFO watermark and full interrupts
 * - Interrupt pin electrical characteristics
 * - Interrupt latching behavior
 */
typedef struct {
    bool drdy_en;    // Data Ready interrupt enable (bit 6)
    bool fwtm_en;    // FIFO watermark interrupt enable (bit 3)
    bool ffull_en;   // FIFO full interrupt enable (bit 4)
    bool int_latch;  // Latching mode enable (bit 2)
    bool int_od;     // Interrupt output type: true for open-drain (bit 0)
    bool int_level;  // Interrupt polarity: true for active high (bit 1)
} bmp390_int_config_t;

// Public function prototypes
esp_err_t bmp390_init(i2c_port_t port);
esp_err_t bmp390_read_sensor_data(double *pressure, double *temperature);
esp_err_t bmp390_read_calib_data(bmp390_calib_data_t *calib);

// Configuration function prototypes
esp_err_t bmp390_get_chip_id(uint8_t *chip_id);
esp_err_t bmp390_get_pwr_ctrl(bmp390_pwr_ctrl_t *ctrl);
esp_err_t bmp390_set_pwr_ctrl(const bmp390_pwr_ctrl_t *ctrl);
esp_err_t bmp390_get_osr(bmp390_osr_settings_t *osr);
esp_err_t bmp390_set_osr(const bmp390_osr_settings_t *osr);
esp_err_t bmp390_get_odr(bmp390_odr_t *odr);
esp_err_t bmp390_set_odr(bmp390_odr_t odr);
esp_err_t bmp390_get_config(bmp390_config_t *config);
esp_err_t bmp390_set_config(const bmp390_config_t *config);
esp_err_t bmp390_get_err_reg(uint8_t *err);
esp_err_t bmp390_get_status(bmp390_status_t *status);
esp_err_t bmp390_get_int_config(bmp390_int_config_t *int_config);
esp_err_t bmp390_set_int_config(const bmp390_int_config_t *int_config);
esp_err_t bmp390_get_rev_id(uint8_t *rev_id);

#endif /* DRIVER_BMP390L_H */ 