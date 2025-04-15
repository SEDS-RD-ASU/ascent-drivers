// definitions yoinked from adafruit bno055 library <3

#ifndef DRIVER_BNO055_H
#define DRIVER_BNO055_H

#include "esp_err.h"
#include "driver/i2c.h"

#define BNO055_I2C_ADDR 0x28

/* PAGE0 REGISTER DEFINITION START*/
#define BNO_CHIP_ID_ADDR                0x00
#define BNO_ACC_REV_ID_ADDR             0x01
#define BNO_MAG_REV_ID_ADDR             0x02
#define BNO_GYR_REV_ID_ADDR             0x03
#define BNO_SW_REV_ID_LSB_ADDR			0x04
#define BNO_SW_REV_ID_MSB_ADDR			0x05
#define BNO_BL_Rev_ID_ADDR			    0X06
#define BNO_Page_ID_ADDR			    0X07

/* Accel data register*/
#define BNO_ACC_DATA_X_LSB_ADDR			0X08
#define BNO_ACC_DATA_X_MSB_ADDR			0X09
#define BNO_ACC_DATA_Y_LSB_ADDR			0X0A
#define BNO_ACC_DATA_Y_MSB_ADDR			0X0B
#define BNO_ACC_DATA_Z_LSB_ADDR			0X0C
#define BNO_ACC_DATA_Z_MSB_ADDR			0X0D

/*Mag data register*/
#define BNO_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO_MAG_DATA_Z_MSB_ADDR			0X13

/*Gyro data registers*/
#define BNO_GYR_DATA_X_LSB_ADDR			0X14
#define BNO_GYR_DATA_X_MSB_ADDR			0X15
#define BNO_GYR_DATA_Y_LSB_ADDR			0X16
#define BNO_GYR_DATA_Y_MSB_ADDR			0X17
#define BNO_GYR_DATA_Z_LSB_ADDR			0X18
#define BNO_GYR_DATA_Z_MSB_ADDR			0X19

/*Euler data registers*/
#define BNO_EUL_HEADING_LSB_ADDR		0X1A
#define BNO_EUL_HEADING_MSB_ADDR		0X1B

#define BNO_EUL_ROLL_LSB_ADDR			0X1C
#define BNO_EUL_ROLL_MSB_ADDR			0X1D

#define BNO_EUL_PITCH_LSB_ADDR			0X1E
#define BNO_EUL_PITCH_MSB_ADDR			0X1F

/*Quaternion data registers*/
#define BNO_QUA_DATA_W_LSB_ADDR			0X20
#define BNO_QUA_DATA_W_MSB_ADDR			0X21
#define BNO_QUA_DATA_X_LSB_ADDR			0X22
#define BNO_QUA_DATA_X_MSB_ADDR			0X23
#define BNO_QUA_DATA_Y_LSB_ADDR			0X24
#define BNO_QUA_DATA_Y_MSB_ADDR			0X25
#define BNO_QUA_DATA_Z_LSB_ADDR			0X26
#define BNO_QUA_DATA_Z_MSB_ADDR			0X27

/* Linear acceleration data registers*/
#define BNO_LIA_DATA_X_LSB_ADDR			0X28
#define BNO_LIA_DATA_X_MSB_ADDR			0X29
#define BNO_LIA_DATA_Y_LSB_ADDR			0X2A
#define BNO_LIA_DATA_Y_MSB_ADDR			0X2B
#define BNO_LIA_DATA_Z_LSB_ADDR			0X2C
#define BNO_LIA_DATA_Z_MSB_ADDR			0X2D

/*Gravity data registers*/
#define BNO_GRV_DATA_X_LSB_ADDR			0X2E
#define BNO_GRV_DATA_X_MSB_ADDR			0X2F
#define BNO_GRV_DATA_Y_LSB_ADDR			0X30
#define BNO_GRV_DATA_Y_MSB_ADDR			0X31
#define BNO_GRV_DATA_Z_LSB_ADDR			0X32
#define BNO_GRV_DATA_Z_MSB_ADDR			0X33

/* Temperature data register*/

#define BNO_TEMP_ADDR				    0X34

/* Status registers*/
#define BNO_CALIB_STAT_ADDR			    0X35
#define BNO_ST_RESULT_ADDR			    0X36
#define BNO_INT_STA_ADDR			    0X37
#define BNO_SYS_STATUS_ADDR			    0X39
#define BNO_SYS_ERR_ADDR			    0X3A

/* Unit selection register*/
#define BNO_UNIT_SEL_ADDR		    	0X3B
#define BNO_DATA_SEL_ADDR		    	0X3C

/* Mode registers*/
#define BNO_OPR_MODE_ADDR		    	0X3D
#define BNO_PWR_MODE_ADDR		    	0X3E

#define BNO_SYS_TRIGGER_ADDR			0X3F
#define BNO_TEMP_SOURCE_ADDR			0X40
/* Axis remap registers*/
#define BNO_AXIS_MAP_CONFIG_ADDR		0X41
#define BNO_AXIS_MAP_SIGN_ADDR			0X42

/* SIC registers*/
#define BNO_SIC_MATRIX_0_LSB_ADDR		0X43
#define BNO_SIC_MATRIX_0_MSB_ADDR		0X44
#define BNO_SIC_MATRIX_1_LSB_ADDR		0X45
#define BNO_SIC_MATRIX_1_MSB_ADDR		0X46
#define BNO_SIC_MATRIX_2_LSB_ADDR		0X47
#define BNO_SIC_MATRIX_2_MSB_ADDR		0X48
#define BNO_SIC_MATRIX_3_LSB_ADDR		0X49
#define BNO_SIC_MATRIX_3_MSB_ADDR		0X4A
#define BNO_SIC_MATRIX_4_LSB_ADDR		0X4B
#define BNO_SIC_MATRIX_4_MSB_ADDR		0X4C
#define BNO_SIC_MATRIX_5_LSB_ADDR		0X4D
#define BNO_SIC_MATRIX_5_MSB_ADDR		0X4E
#define BNO_SIC_MATRIX_6_LSB_ADDR		0X4F
#define BNO_SIC_MATRIX_6_MSB_ADDR		0X50
#define BNO_SIC_MATRIX_7_LSB_ADDR		0X51
#define BNO_SIC_MATRIX_7_MSB_ADDR		0X52
#define BNO_SIC_MATRIX_8_LSB_ADDR		0X53
#define BNO_SIC_MATRIX_8_MSB_ADDR		0X54

/* Accelerometer Offset registers*/
#define ACC_OFFSET_X_LSB_ADDR			0X55
#define ACC_OFFSET_X_MSB_ADDR			0X56
#define ACC_OFFSET_Y_LSB_ADDR			0X57
#define ACC_OFFSET_Y_MSB_ADDR			0X58
#define ACC_OFFSET_Z_LSB_ADDR			0X59
#define ACC_OFFSET_Z_MSB_ADDR			0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR			0X5B
#define MAG_OFFSET_X_MSB_ADDR			0X5C
#define MAG_OFFSET_Y_LSB_ADDR			0X5D
#define MAG_OFFSET_Y_MSB_ADDR			0X5E
#define MAG_OFFSET_Z_LSB_ADDR			0X5F
#define MAG_OFFSET_Z_MSB_ADDR			0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR			0X61
#define GYRO_OFFSET_X_MSB_ADDR			0X62
#define GYRO_OFFSET_Y_LSB_ADDR			0X63
#define GYRO_OFFSET_Y_MSB_ADDR			0X64
#define GYRO_OFFSET_Z_LSB_ADDR			0X65
#define GYRO_OFFSET_Z_MSB_ADDR			0X66

/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACC_CONFIG_ADDR					0X08
#define MAG_CONFIG_ADDR					0X09
#define GYRO_CONFIG_ADDR				0X0A
#define GYRO_MODE_CONFIG_ADDR			0X0B
#define ACC_SLEEP_CONFIG_ADDR			0X0C
#define GYR_SLEEP_CONFIG_ADDR			0X0D
#define MAG_SLEEP_CONFIG_ADDR			0x0E

/* Interrupt registers*/
#define INT_MSK_ADDR					0X0F
#define INT_ADDR						0X10
#define ACC_AM_THRES_ADDR				0X11
#define ACC_INT_SETTINGS_ADDR			0X12
#define ACC_HG_DURATION_ADDR			0X13
#define ACC_HG_THRES_ADDR				0X14
#define ACC_NM_THRES_ADDR				0X15
#define ACC_NM_SET_ADDR					0X16
#define GYR_INT_SETING_ADDR				0X17
#define GYR_HR_X_SET_ADDR				0X18
#define GYR_DUR_X_ADDR					0X19
#define GYR_HR_Y_SET_ADDR				0X1A
#define GYR_DUR_Y_ADDR					0X1B
#define GYR_HR_Z_SET_ADDR				0X1C
#define GYR_DUR_Z_ADDR					0X1D
#define GYR_AM_THRES_ADDR				0X1E
#define GYR_AM_SET_ADDR					0X1F
/* PAGE1 REGISTERS DEFINITION END*/

// Operation modes
typedef enum {
    CONFIG = 0X00,
    ACCONLY = 0X01,
    MAGONLY = 0X02,
    GYRONLY = 0X03,
    ACCMAG = 0X04,
    ACCGYRO = 0X05,
    MAGGYRO = 0X06,
    AMG = 0X07,
    IMUPLUS = 0X08,
    COMPASS = 0X09,
    M4G = 0X0A,
    NDOF_FMC_OFF = 0X0B,
    NDOF = 0X0C
} bno055_opmode_t;

typedef enum {
    NORMAL = 0x00,
    LOWPOWER = 0x01,
    SUSPEND = 0x02,
    INVALID = 0x03
} bno055_powermode_t;

typedef enum {
    TEMP_ACC = 0x0,
    TEMP_GYRO = 0x01
} bno055_tempsource_t;


typedef struct {
    uint8_t x;  // 0 = X-axis, 1 = Y-axis, 2 = Z-axis
    uint8_t y;  // 0 = X-axis, 1 = Y-axis, 2 = Z-axis
    uint8_t z;  // 0 = X-axis, 1 = Y-axis, 2 = Z-axis
} bno_axismap;

typedef enum {
    ACC_C_NORMAL = 0x0, //default
    ACC_C_SUSPEND = 0x01,
    ACC_C_LOWPOWER1 = 0x02,
    ACC_C_STANDBY = 0x03,
    ACC_C_LOWPOWER2 = 0x04,
    ACC_C_DEEPSUSPEND = 0x05
} bno055_acc_pwrmode_t;

typedef enum {
    ACC_C_H7_18 = 0x0,
    ACC_C_H15_63 = 0x01,
    ACC_C_H31_23 = 0x02,
    ACC_C_H62_50 = 0x03, //default
    ACC_C_H125 = 0x04,
    ACC_C_H250 = 0x05,
    ACC_C_H500 = 0x06,
    ACC_C_H1000 = 0x07
} bno055_acc_bandwidth_t;

typedef enum {
    ACC_C_RANGE_2G = 0x0,
    ACC_C_RANGE_4G = 0x01, //default
    ACC_C_RANGE_8G = 0x02,
    ACC_C_RANGE_16G = 0x03
} bno055_acc_range_t;
typedef enum {
    MAG_C_NORMAL = 0x0, //default
    MAG_C_SLEEP = 0x01,
    MAG_C_SUSPEND = 0x02,
    MAG_C_FORCEMODE = 0x03
} bno055_mag_pwrmode_t;

typedef enum {
    MAG_C_LOWPOWER = 0x0,
    MAG_C_REGULAR = 0x01, //default
    MAG_C_ENHANCED = 0x02,
    MAG_C_HIGHACCURACY = 0x03
} bno055_mag_oprmode_t;

typedef enum {
    MAG_C_H2 = 0x0,
    MAG_C_H6 = 0x01,
    MAG_C_H8 = 0x02,
    MAG_C_H10 = 0x03, //defualt
    MAG_C_H15 = 0x04,
    MAG_C_H20 = 0x05,
    MAG_C_H25 = 0x06,
    MAG_C_H30 = 0x07
} bno055_mag_datarate_t;

typedef enum {
    GYRO_C_D2000 = 0x0, //default
    GYRO_C_D1000 = 0x01,
    GYRO_C_D500 = 0x02,
    GYRO_C_D250 = 0x03,
    GYRO_C_D125 = 0x04
} bno055_gyro_range_t;

typedef enum {
    GYRO_C_H523 = 0x0,
    GYRO_C_H230 = 0x01,
    GYRO_C_H116 = 0x02,
    GYRO_C_H47 = 0x03,
    GYRO_C_H23 = 0x04,
    GYRO_C_H12 = 0x05,
    GYRO_C_H64 = 0x06,
    GYRO_C_H32 = 0x07 //default
} bno055_gyro_bandwidth_t;

typedef enum {
    GYRO_C_NORMAL = 0x0, //default
    GYRO_C_FASTPOWERUP = 0x01,
    GYRO_C_DEEPSUSPEND = 0x02,
    GYRO_C_SUSPEND = 0x03,
    GYRO_C_ADVANCEPOWERSAVE = 0x04
} bno055_gyro_powermode_t;

typedef struct {
    int16_t Matrix1;
    int16_t Matrix2;
    int16_t Matrix3;
    int16_t Matrix4;
    int16_t Matrix5;
    int16_t Matrix6;
    int16_t Matrix7;
    int16_t Matrix8;
    int16_t Matrix9;
} bno055_sic_matrix_t;

/**
 * @brief Initialize the BNO055 sensor
 * 
 * @param port The I2C port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bno055_init(i2c_port_t port);

/**
 * @brief Set the operation mode of the BNO055
 * 
 * @param mode The desired operation mode
 */
void bno_setoprmode(bno055_opmode_t mode);

/**
 * @brief Get calibration status
 * 
 * @param sys System calibration status
 * @param gyro Gyroscope calibration status
 * @param accel Accelerometer calibration status
 * @param mag Magnetometer calibration status
 */
void bno_get_calib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

/**
 * @brief Read a register from the BNO055
 * 
 * @param reg_addr The address of the register to read
 * @return uint8_t The value read from the register
 */
uint8_t bnoreadRegister(uint8_t reg_addr);

/**
 * @brief Read accelerometer, magnetometer, and gyroscope data
 * 
 * @param acc_x Pointer to store accelerometer X data
 * @param acc_y Pointer to store accelerometer Y data
 * @param acc_z Pointer to store accelerometer Z data
 * @param mag_x Pointer to store magnetometer X data
 * @param mag_y Pointer to store magnetometer Y data
 * @param mag_z Pointer to store magnetometer Z data
 * @param gyr_x Pointer to store gyroscope X data
 * @param gyr_y Pointer to store gyroscope Y data
 * @param gyr_z Pointer to store gyroscope Z data
 */
void bno_readamg(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                 int16_t *mag_x, int16_t *mag_y, int16_t *mag_z,
                 int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z);

/**
 * @brief Get the current page of the BNO055
 * 
 * @return uint8_t The current page number
 */
uint8_t bno_getpage(void);

/**
 * @brief Read Euler angles from the BNO055
 * 
 * @param eul_heading Pointer to store the heading value
 * @param eul_roll Pointer to store the roll value
 * @param eul_pitch Pointer to store the pitch value
 */
void bno_readeuler(int16_t *eul_heading, int16_t *eul_roll, int16_t *eul_pitch);

/**
 * @brief Read multiple registers from the BNO055
 * 
 * @param reg_addr The address of the first register to read
 * @param length The number of registers to read
 * @return uint8_t* Pointer to the data read from the registers
 */
uint8_t* bnoreadMultiple(uint8_t reg_addr, size_t length);

/**
 * @brief Write a value to a register of the BNO055
 * 
 * @param reg_addr The address of the register to write to
 * @param data The value to write
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bnowriteRegister(uint8_t reg_addr, uint8_t data);

/**
 * @brief Get the current operation mode of the BNO055
 * 
 * @return uint8_t The current operation mode
 */
bno055_opmode_t bno055_get_oprmode(void);

/**
 * @brief Read quaternion data from the BNO055
 * 
 * @param quart_w Pointer to store the W component
 * @param quart_x Pointer to store the X component
 * @param quart_y Pointer to store the Y component
 * @param quart_z Pointer to store the Z component
 */
void bno_readquart(int16_t *quart_w, int16_t *quart_x, int16_t *quart_y, int16_t *quart_z);

/**
 * @brief Read linear acceleration data from the BNO055
 * 
 * @param lia_x Pointer to store the linear acceleration X data
 * @param lia_y Pointer to store the linear acceleration Y data
 * @param lia_z Pointer to store the linear acceleration Z data
 */
void bno_readlia(int16_t *lia_x, int16_t *lia_y, int16_t *lia_z);

/**
 * @brief Read gravity data from the BNO055
 * 
 * @param grav_x Pointer to store the gravity X data
 * @param grav_y Pointer to store the gravity Y data
 * @param grav_z Pointer to store the gravity Z data
 */
void bno_readgrav(int16_t *grav_x, int16_t *grav_y, int16_t *grav_z);

/**
 * @brief Get the temperature from the BNO055
 * 
 * @return uint8_t The temperature value
 */
uint8_t bno_gettemp(void);

/**
 * @brief Set the page of the BNO055
 * 
 * @param page The page number to set
 */
void bno_setpage(int8_t page);

void bno_get_units(bool *ORI_android_windows, bool *temp_unit, bool *eul_unit, bool *gyr_unit, bool *acc_unit);
void bno_getselftest(bool *st_mcu, bool *st_gyr, bool *st_mag, bool *st_acc);
bool bno_getclockstatus(void);
uint8_t bno_getsysstatus(void);
uint8_t bno_getsyserror(void);
bno055_powermode_t bno_getpowermode(void);
void bno_setpowermode(bno055_powermode_t mode);
void bno_trigger_st(void);
void bno_trigger_rst(void);
void bno_trigger_int_rst(void);
bno055_tempsource_t bno_get_tempsource(void);
void bno_set_tempsource(bno055_tempsource_t source);

void bno_getinterruptstatus(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy);
void bno_getinterruptmask(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy);
void bno_setinterruptmask(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy);
void bno_getinterruptenable(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy);
void bno_setinterruptenable(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy);

bno_axismap bno_get_axismapconfig(void);
void bno_set_axismapconfig(bno_axismap axis_map);
void bno_get_axismapsign(bool *x, bool *y, bool *z);
void bno_set_axismapsign(bool x, bool y, bool z);

void bno_configure_acc(bno055_acc_pwrmode_t pwr, bno055_acc_bandwidth_t bandwidth, bno055_acc_range_t range);
void bno_config_mag(bno055_mag_pwrmode_t pwr, bno055_mag_oprmode_t oprmode, bno055_mag_datarate_t datarate);
void bno_configure_gyro(bno055_gyro_range_t range, bno055_gyro_bandwidth_t bandwidth, bno055_gyro_powermode_t pwr);

void bno_getacc_config(bno055_acc_pwrmode_t *pwr, bno055_acc_bandwidth_t *bandwidth, bno055_acc_range_t *range);
void bno_getmag_config(bno055_mag_pwrmode_t *pwr, bno055_mag_oprmode_t *oprmode, bno055_mag_datarate_t *datarate);
void bno_getgyro_config(bno055_gyro_range_t *range, bno055_gyro_bandwidth_t *bandwidth, bno055_gyro_powermode_t *pwr);

void bno_get_acc_amthres(uint8_t *am_thres);
void bno_set_acc_amthres(uint8_t am_thres);
void bno_get_acc_int(bool *hg_z, bool *hg_y, bool *hg_x, bool *am_nm_z, bool *am_nm_y, bool *am_nm_x, uint8_t *am_dur);
void bno_set_acc_int(bool hg_z, bool hg_y, bool hg_x, bool am_nm_z, bool am_nm_y, bool am_nm_x, uint8_t am_dur);
void bno_get_acc_hgduration(uint8_t *acc_hg_duration);
void bno_set_acc_hgduration(uint8_t acc_hg_duration);
void bno_get_acc_hgtresh(uint8_t *acc_hg_tresh);
void bno_set_acc_hgtresh(uint8_t acc_hg_tresh);
void bno_get_acc_nmtresh(uint8_t *acc_nm_tresh);
void bno_set_acc_nmtresh(uint8_t acc_nm_tresh);
void bno_get_acc_nm_set(bool *slowmotion_nomotion, uint8_t *slow_no_mot_dur);
void bno_set_acc_nm_set(bool slowmotion_nomotion, uint8_t slow_no_mot_dur);

void bno_get_gyro_int_setting(bool *hr_filt, bool *am_filt, bool *hr_z_axis, bool *hr_y_axis, bool *hr_x_axis, bool *am_z_axis, bool *am_y_axis, bool *am_x_axis);
void bno_set_gyro_int_setting(bool hr_filt, bool am_filt, bool hr_z_axis, bool hr_y_axis, bool hr_x_axis, bool am_z_axis, bool am_y_axis, bool am_x_axis);
void bno_get_gyr_hr_x_set(uint8_t *hr_x_thres_hyst, uint8_t *hr_x_threshold);
void bno_set_gyr_hr_x_set(uint8_t hr_x_thres_hyst, uint8_t hr_x_threshold);
void bno_get_gyr_dur_x(uint8_t *hr_x_duration);
void bno_set_gyr_dur_x(uint8_t hr_x_duration);
void bno_get_gyr_hr_y_set(uint8_t *hr_y_thres_hyst, uint8_t *hr_y_threshold);
void bno_set_gyr_hr_y_set(uint8_t hr_y_thres_hyst, uint8_t hr_y_threshold);
void bno_get_gyr_dur_y(uint8_t *hr_y_duration);
void bno_set_gyr_dur_y(uint8_t hr_y_duration);
void bno_get_gyr_hr_z_set(uint8_t *hr_z_thres_hyst, uint8_t *hr_z_threshold);
void bno_set_gyr_hr_z_set(uint8_t hr_z_thres_hyst, uint8_t hr_z_threshold);
void bno_get_gyr_dur_z(uint8_t *hr_z_duration);
void bno_set_gyr_dur_z(uint8_t hr_z_duration);
void bno_get_gyr_am_thresh(uint8_t *gyr_am_thres);
void bno_set_gyr_am_thresh(uint8_t gyr_am_thres);
void bno_get_gyr_am_set(uint8_t *awake_duration, uint8_t *slope_samples);
void bno_set_gyr_am_set(uint8_t awake_duration, uint8_t slope_samples);

/**
 * @brief Get the SIC matrix from the BNO055
 * 
 * @param matrix Pointer to the structure to store the SIC matrix
 */
void bno_get_sic_matrix(bno055_sic_matrix_t *matrix);

/**
 * @brief Set the SIC matrix in the BNO055
 * 
 * @param matrix The SIC matrix to set
 */
void bno_set_sic_matrix(bno055_sic_matrix_t matrix);

/**
 * @brief Get the accelerometer offsets from the BNO055
 * 
 * @param x_offset Pointer to store the X offset
 * @param y_offset Pointer to store the Y offset
 * @param z_offset Pointer to store the Z offset
 */
void bno_get_acc_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset);

/**
 * @brief Set the accelerometer offsets in the BNO055
 * 
 * @param x_offset The X offset to set
 * @param y_offset The Y offset to set
 * @param z_offset The Z offset to set
 */
void bno_set_acc_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset);

/**
 * @brief Get the magnetometer offsets from the BNO055
 * 
 * @param x_offset Pointer to store the X offset
 * @param y_offset Pointer to store the Y offset
 * @param z_offset Pointer to store the Z offset
 */
void bno_get_mag_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset);

/**
 * @brief Set the magnetometer offsets in the BNO055
 * 
 * @param x_offset The X offset to set
 * @param y_offset The Y offset to set
 * @param z_offset The Z offset to set
 */
void bno_set_mag_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset);

/**
 * @brief Get the gyroscope offsets from the BNO055
 * 
 * @param x_offset Pointer to store the X offset
 * @param y_offset Pointer to store the Y offset
 * @param z_offset Pointer to store the Z offset
 */
void bno_get_gyro_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset);

/**
 * @brief Set the gyroscope offsets in the BNO055
 * 
 * @param x_offset The X offset to set
 * @param y_offset The Y offset to set
 * @param z_offset The Z offset to set
 */
void bno_set_gyro_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset);

/**
 * @brief Get the accelerometer radius from the BNO055
 * 
 * @param acc_radius Pointer to store the accelerometer radius
 */
void bno_get_acc_radius(uint16_t *acc_radius);

/**
 * @brief Set the accelerometer radius in the BNO055
 * 
 * @param acc_radius The accelerometer radius to set
 */
void bno_set_acc_radius(uint16_t acc_radius);

/**
 * @brief Get the magnetometer radius from the BNO055
 * 
 * @param mag_radius Pointer to store the magnetometer radius
 */
void bno_get_mag_radius(uint16_t *mag_radius);

/**
 * @brief Set the magnetometer radius in the BNO055
 * 
 * @param mag_radius The magnetometer radius to set
 */
void bno_set_mag_radius(uint16_t mag_radius);

#endif /* DRIVER_BNO055_H */