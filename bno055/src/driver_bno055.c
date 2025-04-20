// initial driver that reads and writes registers, and initializes the sensor to read data

#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver_bno055.h"
#include "i2c_manager.h"

static const char *TAG = "BNO055";

static int i2c_port;

esp_err_t bno055_init(i2c_port_t port) {
    // Store only the port number
    i2c_port = port;
    
    // Check if I2C is already initialized
    if (!i2c_manager_is_initialized(port)) {
        ESP_LOGE(TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    bno_setpage(0);

    return ESP_OK;
}

uint8_t bnoreadRegister(uint8_t reg_addr) {
    uint8_t data;
    i2c_manager_read_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
    return data;
}

uint8_t* bnoreadMultiple(uint8_t reg_addr, size_t length) {
    uint8_t *data = malloc(length); // Allocate memory for the data
    if (data != NULL) {
        i2c_manager_read_register(i2c_port, BNO055_I2C_ADDR, reg_addr, data, length);
    }
    return data; // Return the pointer to the data
}

esp_err_t bnowriteRegister(uint8_t reg_addr, uint8_t data) {
    return i2c_manager_write_register(i2c_port, BNO055_I2C_ADDR, reg_addr, &data, 1);
}

uint8_t bno_getoprmode(void) {
    bno_setpage(0);
    bno055_opmode_t currentmode = bnoreadRegister(BNO_OPR_MODE_ADDR);
    return currentmode;
}

bno055_powermode_t bno_getpowermode(void) {
    bno_setpage(0);
    bno055_powermode_t powermode = bnoreadRegister(BNO_PWR_MODE_ADDR) & 0x03;
    return powermode;
}

void bno_setoprmode(bno055_opmode_t mode) {
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bno_setpage(0);
        bnowriteRegister(BNO_OPR_MODE_ADDR, mode);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        uint8_t current_mode = bno_getoprmode();
        if (current_mode == mode) {
            return; // Exit if the mode is set correctly
        }
    }
    
    ESP_LOGE(TAG, "Failed to set mode to %d after 3 attempts", mode);
}

void bno_setpowermode(bno055_powermode_t mode) {
    // Attempt to set the mode and verify it
    for (int i = 0; i < 3; i++) {
        bno_setpage(0);
        bnowriteRegister(BNO_PWR_MODE_ADDR, mode);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        uint8_t current_mode = bno_getpowermode();
        if (current_mode == mode) {
            return; // Exit if the mode is set correctly
        }
    }

    ESP_LOGE(TAG, "Failed to set power mode to %d after 3 attempts", mode);
}

void bno_get_calib(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    bno_setpage(0);
    uint8_t calData = bnoreadRegister(BNO_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

void bno_readamg(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                 int16_t *mag_x, int16_t *mag_y, int16_t *mag_z,
                 int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x08, 18); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {
        // Parse accelerometer data
        if (acc_x) *acc_x = (int16_t)((data[1] << 8) | data[0]);
        if (acc_y) *acc_y = (int16_t)((data[3] << 8) | data[2]);
        if (acc_z) *acc_z = (int16_t)((data[5] << 8) | data[4]);

        // Parse magnetometer data
        if (mag_x) *mag_x = (int16_t)((data[7] << 8) | data[6]);
        if (mag_y) *mag_y = (int16_t)((data[9] << 8) | data[8]);
        if (mag_z) *mag_z = (int16_t)((data[11] << 8) | data[10]);

        // Parse gyroscope data
        if (gyr_x) *gyr_x = (int16_t)((data[13] << 8) | data[12]);
        if (gyr_y) *gyr_y = (int16_t)((data[15] << 8) | data[14]);
        if (gyr_z) *gyr_z = (int16_t)((data[17] << 8) | data[16]);

        free(data); // Free the allocated memory
    }
}

void bno_readacc(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x08, 18); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {
        // Parse accelerometer data
        if (acc_x) *acc_x = (int16_t)((data[1] << 8) | data[0]);
        if (acc_y) *acc_y = (int16_t)((data[3] << 8) | data[2]);
        if (acc_z) *acc_z = (int16_t)((data[5] << 8) | data[4]);

        free(data); // Free the allocated memory
    }
}

void bno_readmag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x08, 18); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {

        // Parse magnetometer data
        if (mag_x) *mag_x = (int16_t)((data[7] << 8) | data[6]);
        if (mag_y) *mag_y = (int16_t)((data[9] << 8) | data[8]);
        if (mag_z) *mag_z = (int16_t)((data[11] << 8) | data[10]);

        free(data); // Free the allocated memory
    }
}
void bno_readgyro(int16_t *gyr_x, int16_t *gyr_y, int16_t *gyr_z) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x08, 18); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {

        // Parse gyroscope data
        if (gyr_x) *gyr_x = (int16_t)((data[13] << 8) | data[12]);
        if (gyr_y) *gyr_y = (int16_t)((data[15] << 8) | data[14]);
        if (gyr_z) *gyr_z = (int16_t)((data[17] << 8) | data[16]);

        free(data); // Free the allocated memory
    }
}

void bno_readeuler(int16_t *eul_heading,
                 int16_t *eul_roll,
                 int16_t *eul_pitch) {
    bno_setpage(0);
    uint8_t *data = bnoreadMultiple(0x1A, 6); // Read 18 registers from 0x08 to 0x19
    if (data != NULL) {
        // Parse heading
        if (eul_heading) *eul_heading = (int16_t)((data[1] << 8) | data[0]);

        // Parse roll
        if (eul_roll) *eul_roll = (int16_t)((data[3] << 8) | data[2]);

        // Parse pitch
        if (eul_pitch) *eul_pitch = (int16_t)((data[5] << 8) | data[4]);

        free(data); // Free the allocated memory
    }
}

void bno_readquart(int16_t *quart_w, int16_t *quart_x, int16_t *quart_y, int16_t *quart_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(0x20, 8);
    if (data != NULL) {
        if (quart_w) *quart_w = (int16_t)((data[1] << 8) | data[0]);
        if (quart_x) *quart_x = (int16_t)((data[3] << 8) | data[2]);
        if (quart_y) *quart_y = (int16_t)((data[5] << 8) | data[4]);
        if (quart_z) *quart_z = (int16_t)((data[7] << 8) | data[6]);

        free(data);
    }
}

void bno_readlia(int16_t *lia_x, int16_t *lia_y, int16_t *lia_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(0x28,6);
    if (data != NULL) {
        // Parse linear acceleration data
        if (lia_x) *lia_x = (int16_t)((data[1] << 8) | data[0]);
        if (lia_y) *lia_y = (int16_t)((data[3] << 8) | data[2]);
        if (lia_z) *lia_z = (int16_t)((data[5] << 8) | data[4]);

        free(data); // Free the allocated memory
    }
}

void bno_readgrav(int16_t *grav_x, int16_t *grav_y, int16_t *grav_z) {
    bno_setpage(0);

    uint8_t *data = bnoreadMultiple(BNO_GRV_DATA_X_MSB_ADDR, 6);
    if (data != NULL) {
    // Parse gravity acceleration data
    if (grav_x) *grav_x = (int16_t)((data[1] << 8) | data[0]);
    if (grav_y) *grav_y = (int16_t)((data[3] << 8) | data[2]);
    if (grav_z) *grav_z = (int16_t)((data[5] << 8) | data[4]);

    free(data);
    }
}

void bno_getselftest(bool *st_mcu, bool *st_gyr, bool *st_mag, bool *st_acc) {
    bno_setpage(0);

    uint8_t st_results = bnoreadRegister(BNO_ST_RESULT_ADDR);

    if (st_mcu) *st_mcu = (st_results >> 0) & 0x01;
    if (st_gyr) *st_gyr = (st_results >> 1) & 0x01;
    if (st_mag) *st_mag = (st_results >> 2) & 0x01;
    if (st_acc) *st_acc = (st_results >> 3) & 0x01;
}

void bno_getinterruptstatus(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(0);

    uint8_t interrupts = bnoreadRegister(BNO_INT_STA_ADDR);

    if(acc_nm) *acc_nm = (interrupts >> 7) & 0x01;
    if(acc_am) *acc_am = (interrupts >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (interrupts >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (interrupts >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (interrupts >> 3) & 0x01;
    if(gyro_am) *gyro_am = (interrupts >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (interrupts >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (interrupts >> 0) & 0x01; 
}

void bno_getinterruptmask(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t mask = bnoreadRegister(0x0F);

    if(acc_nm) *acc_nm = (mask >> 7) & 0x01;
    if(acc_am) *acc_am = (mask >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (mask >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (mask >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (mask >> 3) & 0x01;
    if(gyro_am) *gyro_am = (mask >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (mask >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (mask >> 0) & 0x01; 
}

void bno_setinterruptmask(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t mask = 0;
    if(acc_nm) mask |= (1 << 7);
    if(acc_am) mask |= (1 << 6);
    if(acc_high_g) mask |= (1 << 5);
    if(gyr_drdy) mask |= (1 << 4);
    if(gyr_high_ratem) mask |= (1 << 3);
    if(gyro_am) mask |= (1 << 2);
    if(mag_drdy) mask |= (1 << 1);
    if(acc_bsx_drdy) mask |= (1 << 0);

    bnowriteRegister(INT_MSK_ADDR, mask);
}

void bno_getinterruptenable(bool *acc_nm, bool *acc_am, bool *acc_high_g, bool *gyr_drdy, bool *gyr_high_ratem, bool *gyro_am, bool *mag_drdy, bool *acc_bsx_drdy) {
    bno_setpage(1);

    uint8_t enabled = bnoreadRegister(INT_ADDR);

    if(acc_nm) *acc_nm = (enabled >> 7) & 0x01;
    if(acc_am) *acc_am = (enabled >> 6) & 0x01;
    if(acc_high_g) *acc_high_g = (enabled >> 5) & 0x01;
    if(gyr_drdy) *gyr_drdy = (enabled >> 4) & 0x01;
    if(gyr_high_ratem) *gyr_high_ratem = (enabled >> 3) & 0x01;
    if(gyro_am) *gyro_am = (enabled >> 2) & 0x01;
    if(mag_drdy) *mag_drdy = (enabled >> 1) & 0x01;
    if(acc_bsx_drdy) *acc_bsx_drdy = (enabled >> 0) & 0x01; 
}

void bno_setinterruptenable(bool acc_nm, bool acc_am, bool acc_high_g, bool gyr_drdy, bool gyr_high_ratem, bool gyro_am, bool mag_drdy, bool acc_bsx_drdy) {
    bno_setpage(1);
    
    uint8_t enabled = 0;
    if(acc_nm) enabled |= (1 << 7);
    if(acc_am) enabled |= (1 << 6);
    if(acc_high_g) enabled |= (1 << 5);
    if(gyr_drdy) enabled |= (1 << 4);
    if(gyr_high_ratem) enabled |= (1 << 3);
    if(gyro_am) enabled |= (1 << 2);
    if(mag_drdy) enabled |= (1 << 1);
    if(acc_bsx_drdy) enabled |= (1 << 0);

    bnowriteRegister(INT_ADDR, enabled);
}

bool bno_getclockstatus(void){
    bno_setpage(0);

    uint8_t sys_clk_status = bnoreadRegister(0x38);

    return ((sys_clk_status >> 1) & 0x01);
}

uint8_t bno_getsysstatus(void){
    bno_setpage(0);

    return bnoreadRegister(BNO_SYS_STATUS_ADDR);
}

uint8_t bno_getsyserror(void){
    bno_setpage(0);

    return bnoreadRegister(BNO_SYS_ERR_ADDR);
}

void bno_get_units(bool *ORI_android_windows, bool *temp_unit, bool *eul_unit, bool *gyr_unit, bool *acc_unit) {
    bno_setpage(0);

    uint8_t units = bnoreadRegister(BNO_UNIT_SEL_ADDR);

    if (ORI_android_windows) *ORI_android_windows = (units & 0x80) != 0;
    if (temp_unit) *temp_unit = (units & 0x10) != 0;
    if (eul_unit) *eul_unit = (units & 0x08) != 0;
    if (gyr_unit) *gyr_unit = (units & 0x04) != 0;
    if (acc_unit) *acc_unit = (units & 0x02) != 0;
}

uint8_t bno_gettemp(void) {
    bno_setpage(0);

    return bnoreadRegister(BNO_TEMP_ADDR);
}

void bno_trigger_st(void) {
    bno_setpage(0);

    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x01);
}

void bno_trigger_rst(void) {
    bno_setpage(0);

    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x20);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void bno_trigger_int_rst(void) {
    bno_setpage(0);
    
    bnowriteRegister(BNO_SYS_TRIGGER_ADDR,0x40);
}



bno055_tempsource_t bno_get_tempsource(void) {
    bno_setpage(0);

    return bnoreadRegister(0x40) & 0x01;
}

void bno_set_tempsource(bno055_tempsource_t source) {
    bno_setpage(0);
    
    // Attempt to set the temperature source and verify it
    for (int i = 0; i < 3; i++) {
        bnowriteRegister(0x40, source);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        
        bno055_tempsource_t current_source = bno_get_tempsource();
        if (current_source == source) {
            return; // Exit if the temperature source is set correctly
        }
    }

    ESP_LOGE(TAG, "Failed to set temp source to %d after 3 attempts", source);
}

bno_axismap bno_get_axismapconfig(void) {
    bno_setpage(0);

    bno_axismap axis_map;
    uint8_t axis_map_reg = bnoreadRegister(0x41);

    axis_map.x = (axis_map_reg & 0x03);        // Bits 1-0: X-axis mapping
    axis_map.y = (axis_map_reg & 0x0C) >> 2;   // Bits 3-2: Y-axis mapping
    axis_map.z = (axis_map_reg & 0x30) >> 4;   // Bits 5-4: Z-axis mapping

    return axis_map;
}

void bno_set_axismapconfig(bno_axismap axis_map) {
    bno_setpage(0);

    // Ensure all values are valid (0=X, 1=Y, 2=Z) and no duplicates
    if (axis_map.x > 2 || axis_map.y > 2 || axis_map.z > 2 ||
        axis_map.x == axis_map.y || axis_map.x == axis_map.z || axis_map.y == axis_map.z) {
        ESP_LOGE(TAG, "Invalid axis mapping configuration!");
        return;
    }

    uint8_t axis_map_reg = 0x00;
    axis_map_reg |= (axis_map.x & 0x03);       
    axis_map_reg |= (axis_map.y & 0x03) << 2;  
    axis_map_reg |= (axis_map.z & 0x03) << 4;  

    // Write and verify
    bnowriteRegister(0x41, axis_map_reg);
    uint8_t verify = bnoreadRegister(0x41);
    if (verify != axis_map_reg) {
        ESP_LOGE(TAG, "Axis map configuration mismatch: expected 0x%02X, got 0x%02X", axis_map_reg, verify);
    }
}

void bno_get_axismapsign(bool *x, bool *y, bool *z) {
    bno_setpage(0);

    uint8_t signs = bnoreadRegister(0x42);

    if (x) *x = (signs & 0x01) != 0;
    if (y) *y = (signs & 0x02) != 0;
    if (z) *z = (signs & 0x04) != 0;
}

void bno_set_axismapsign(bool x, bool y, bool z) {
    bno_setpage(0);

    uint8_t signs = 0x00;
    if (x) signs |= 0x01;
    if (y) signs |= 0x02;
    if (z) signs |= 0x04;

    bnowriteRegister(0x42, signs);
    uint8_t verify = bnoreadRegister(0x42);
    if (verify != signs) {
        ESP_LOGE(TAG, "Axis sign configuration mismatch: expected 0x%02X, got 0x%02X", signs, verify);
    }
}

void bno_configure_acc(bno055_acc_pwrmode_t pwr, bno055_acc_bandwidth_t bandwidth, bno055_acc_range_t range) {
    uint8_t config;
    bno_setpage(1);
    config = (pwr << 5) | (bandwidth << 3) | range;
    bnowriteRegister(ACC_CONFIG_ADDR,config);
}

void bno_config_mag(bno055_mag_pwrmode_t pwr, bno055_mag_oprmode_t oprmode, bno055_mag_datarate_t datarate) {
    uint8_t config;
    bno_setpage(1);
    config = (pwr << 5) | (oprmode << 3) | datarate;
    bnowriteRegister(MAG_CONFIG_ADDR,config);
}

void bno_configure_gyro(bno055_gyro_range_t range, bno055_gyro_bandwidth_t bandwidth, bno055_gyro_powermode_t pwr) {
    uint8_t config_0;
    uint8_t config_1;
    bno_setpage(1);

    config_0 = (bandwidth << 4) | range;
    config_1 = pwr;

    bnowriteRegister(GYRO_CONFIG_ADDR,config_0);
    bnowriteRegister(GYRO_MODE_CONFIG_ADDR,config_1);
}

void bno_getacc_config(bno055_acc_pwrmode_t *pwr, bno055_acc_bandwidth_t *bandwidth, bno055_acc_range_t *range) {
    bno_setpage(1);
    uint8_t currentconfig = bnoreadRegister(ACC_CONFIG_ADDR);

    if (pwr) *pwr = (bno055_acc_pwrmode_t)(currentconfig >> 5);
    if (bandwidth) *bandwidth = (bno055_acc_bandwidth_t)(currentconfig >> 3) & 0x03;
    if (range) *range = (bno055_acc_range_t)(currentconfig & 0x03);
}

void bno_getmag_config(bno055_mag_pwrmode_t *pwr, bno055_mag_oprmode_t *oprmode, bno055_mag_datarate_t *datarate) {
    bno_setpage(1);
    uint8_t currentconfig = bnoreadRegister(MAG_CONFIG_ADDR);

    if (pwr) *pwr = (bno055_mag_pwrmode_t)(currentconfig >> 5);
    if (oprmode) *oprmode = (bno055_mag_oprmode_t)(currentconfig >> 3) & 0x03;
    if (datarate) *datarate = (bno055_mag_datarate_t)(currentconfig & 0x07);
}

void bno_getgyro_config(bno055_gyro_range_t *range, bno055_gyro_bandwidth_t *bandwidth, bno055_gyro_powermode_t *pwr) {
    bno_setpage(1);
    uint8_t config_0 = bnoreadRegister(GYRO_CONFIG_ADDR);
    uint8_t config_1 = bnoreadRegister(GYRO_MODE_CONFIG_ADDR);

    if (range) *range = (bno055_gyro_range_t)(config_0 & 0x0F);
    if (bandwidth) *bandwidth = (bno055_gyro_bandwidth_t)(config_0 >> 4);
    if (pwr) *pwr = (bno055_gyro_powermode_t)(config_1);
}

void bno_get_acc_amthres(uint8_t *am_thres) {
    bno_setpage(1);
    if (am_thres) *am_thres = bnoreadRegister(0x11);
}

void bno_set_acc_amthres(uint8_t am_thres) {
    bno_setpage(1);
    bnowriteRegister(0x11,am_thres);
}

void bno_get_acc_int(bool *hg_z, bool *hg_y, bool *hg_x, bool *am_nm_z, bool *am_nm_y, bool *am_nm_x, uint8_t *am_dur) {
    bno_setpage(1);
    uint8_t acc_int = bnoreadRegister(0x12);

    if (hg_z) *hg_z = (acc_int & 0x80) != 0;  // High-G interrupt Z-axis
    if (hg_y) *hg_y = (acc_int & 0x40) != 0;  // High-G interrupt Y-axis
    if (hg_x) *hg_x = (acc_int & 0x20) != 0;  // High-G interrupt X-axis
    if (am_nm_z) *am_nm_z = (acc_int & 0x10) != 0;  // AM/NM Z-axis
    if (am_nm_y) *am_nm_y = (acc_int & 0x08) != 0;  // AM/NM Y-axis
    if (am_nm_x) *am_nm_x = (acc_int & 0x04) != 0;  // AM/NM X-axis
    if (am_dur) *am_dur = (acc_int & 0x03);
}

void bno_set_acc_int(bool hg_z, bool hg_y, bool hg_x, bool am_nm_z, bool am_nm_y, bool am_nm_x, uint8_t am_dur) {
    bno_setpage(1);
    uint8_t acc_int = 0;

    if (hg_z) acc_int |= 0x80;  // High-G interrupt Z-axis
    if (hg_y) acc_int |= 0x40;  // High-G interrupt Y-axis
    if (hg_x) acc_int |= 0x20;  // High-G interrupt X-axis
    if (am_nm_z) acc_int |= 0x10;  // AM/NM Z-axis
    if (am_nm_y) acc_int |= 0x08;  // AM/NM Y-axis
    if (am_nm_x) acc_int |= 0x04;  // AM/NM X-axis
    acc_int |= (am_dur & 0x03);

    bnowriteRegister(0x12, acc_int);
}

void bno_get_acc_hgduration(uint8_t *acc_hg_duration) {
    bno_setpage(1);
    if (acc_hg_duration) *acc_hg_duration = bnoreadRegister(0x13);
}

void bno_set_acc_hgduration(uint8_t acc_hg_duration) {
    bno_setpage(1);
    bnowriteRegister(0x13,acc_hg_duration);
}

void bno_get_acc_hgtresh(uint8_t *acc_hg_tresh) {
    bno_setpage(1);
    if(acc_hg_tresh) *acc_hg_tresh = bnoreadRegister(0x14);
}

void bno_set_acc_hgtresh(uint8_t acc_hg_tresh) {
    bno_setpage(1);
    bnowriteRegister(0x14,acc_hg_tresh);
}

void bno_get_acc_nmtresh(uint8_t *acc_nm_tresh) {
    bno_setpage(1);
    if(acc_nm_tresh) *acc_nm_tresh = bnoreadRegister(0x14);
}

void bno_set_acc_nmtresh(uint8_t acc_nm_tresh) {
    bno_setpage(1);
    bnowriteRegister(0x14,acc_nm_tresh);
}

void bno_get_acc_nm_set(bool *slowmotion_nomotion, uint8_t *slow_no_mot_dur) {
    bno_setpage(1);
    uint8_t acc_nm_set = bnoreadRegister(0x16);

    if (slowmotion_nomotion) *slowmotion_nomotion = (acc_nm_set & 0x01) != 0;
    if (slow_no_mot_dur) *slowmotion_nomotion = (acc_nm_set >> 1);
}

void bno_set_acc_nm_set(bool slowmotion_nomotion, uint8_t slow_no_mot_dur) {
    bno_setpage(1);
    uint8_t acc_nm_set = 0;

    if (slowmotion_nomotion) acc_nm_set |= 0x01;
    acc_nm_set |= (slow_no_mot_dur << 1);

    bnowriteRegister(0x16, acc_nm_set);
}

void bno_get_gyro_int_setting(bool *hr_filt, bool *am_filt, bool *hr_z_axis, bool *hr_y_axis, bool *hr_x_axis, bool *am_z_axis, bool *am_y_axis, bool *am_x_axis) {
    bno_setpage(1);

    uint8_t gyr_int_settings = bnoreadRegister(0x17);

    if (hr_filt) *hr_filt = (gyr_int_settings & 0x80) != 0; 
    if (am_filt) *am_filt = (gyr_int_settings & 0x40) != 0; 
    if (hr_z_axis) *hr_z_axis = (gyr_int_settings & 0x20) != 0; 
    if (hr_y_axis) *hr_y_axis = (gyr_int_settings & 0x10) != 0; 
    if (hr_x_axis) *hr_x_axis = (gyr_int_settings & 0x08) != 0; 
    if (am_z_axis) *am_z_axis = (gyr_int_settings & 0x04) != 0; 
    if (am_y_axis) *am_y_axis = (gyr_int_settings & 0x02) != 0; 
    if (am_x_axis) *am_x_axis = (gyr_int_settings & 0x01) != 0; 
}

void bno_set_gyro_int_setting(bool hr_filt, bool am_filt, bool hr_z_axis, bool hr_y_axis, bool hr_x_axis, bool am_z_axis, bool am_y_axis, bool am_x_axis) {
    bno_setpage(1);

    uint8_t gyr_int_settings = 0;

    if (hr_filt) gyr_int_settings |= 0x80;
    if (am_filt) gyr_int_settings |= 0x40;
    if (hr_z_axis) gyr_int_settings |= 0x20;
    if (hr_y_axis) gyr_int_settings |= 0x10;
    if (hr_x_axis) gyr_int_settings |= 0x08;
    if (am_z_axis) gyr_int_settings |= 0x04;
    if (am_y_axis) gyr_int_settings |= 0x02;
    if (am_x_axis) gyr_int_settings |= 0x01;

    bnowriteRegister(0x17, gyr_int_settings);
}

void bno_get_gyr_hr_x_set(uint8_t *hr_x_thres_hyst, uint8_t *hr_x_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_x_set = bnoreadRegister(0x18);

    if (hr_x_thres_hyst) *hr_x_thres_hyst = (gyr_hr_x_set >> 5);
    if (hr_x_threshold) *hr_x_threshold = (gyr_hr_x_set & 0x1F);
}

void bno_set_gyr_hr_x_set(uint8_t hr_x_thres_hyst, uint8_t hr_x_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_x_set = 0;

    gyr_hr_x_set |= (hr_x_thres_hyst << 5);
    gyr_hr_x_set |= (hr_x_threshold & 0x1F);

    bnowriteRegister(0x18, gyr_hr_x_set);
}

void bno_get_gyr_dur_x(uint8_t *hr_x_duration){ 
    bno_setpage(0);
    if (hr_x_duration) *hr_x_duration = bnoreadRegister(0x19);
}

void bno_set_gyr_dur_x(uint8_t hr_x_duration) {
    bno_setpage(1);
    bnowriteRegister(0x19, hr_x_duration);
}

void bno_get_gyr_hr_y_set(uint8_t *hr_y_thres_hyst, uint8_t *hr_y_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_y_set = bnoreadRegister(0x1A);

    if (hr_y_thres_hyst) *hr_y_thres_hyst = (gyr_hr_y_set >> 5);
    if (hr_y_threshold) *hr_y_threshold = (gyr_hr_y_set & 0x1F);
}

void bno_set_gyr_hr_y_set(uint8_t hr_y_thres_hyst, uint8_t hr_y_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_y_set = 0;

    gyr_hr_y_set |= (hr_y_thres_hyst << 5);
    gyr_hr_y_set |= (hr_y_threshold & 0x1F);

    bnowriteRegister(0x1A, gyr_hr_y_set);
}

void bno_get_gyr_dur_y(uint8_t *hr_y_duration){ 
    bno_setpage(0);
    if (hr_y_duration) *hr_y_duration = bnoreadRegister(0x1B);
}

void bno_set_gyr_dur_y(uint8_t hr_y_duration) {
    bno_setpage(1);
    bnowriteRegister(0x1B, hr_y_duration);
}

void bno_get_gyr_hr_z_set(uint8_t *hr_z_thres_hyst, uint8_t *hr_z_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_z_set = bnoreadRegister(0x1C);

    if (hr_z_thres_hyst) *hr_z_thres_hyst = (gyr_hr_z_set >> 5);
    if (hr_z_threshold) *hr_z_threshold = (gyr_hr_z_set & 0x1F);
}

void bno_set_gyr_hr_z_set(uint8_t hr_z_thres_hyst, uint8_t hr_z_threshold) {
    bno_setpage(1);

    uint8_t gyr_hr_z_set = 0;

    gyr_hr_z_set |= (hr_z_thres_hyst << 5);
    gyr_hr_z_set |= (hr_z_threshold & 0x1F);

    bnowriteRegister(0x1C, gyr_hr_z_set);
}

void bno_get_gyr_dur_z(uint8_t *hr_z_duration){ 
    bno_setpage(0);
    if (hr_z_duration) *hr_z_duration = bnoreadRegister(0x1D);
}

void bno_set_gyr_dur_z(uint8_t hr_z_duration) {
    bno_setpage(1);
    bnowriteRegister(0x1D, hr_z_duration);
}

void bno_get_gyr_am_thresh(uint8_t *gyr_am_thres) {
    bno_setpage(1);
    if (gyr_am_thres) *gyr_am_thres = (bnoreadRegister(0x1E) & 0x7F);
}

void bno_set_gyr_am_thresh(uint8_t gyr_am_thres) {
    bno_setpage(1);
    bnowriteRegister(0x1E, gyr_am_thres & 0x7F);
}

void bno_get_gyr_am_set(uint8_t *awake_duration, uint8_t *slope_samples) {
    bno_setpage(1);
    uint8_t gyr_am_set = bnoreadRegister(0x1F);
    if (awake_duration) *awake_duration = (gyr_am_set >> 2);
    if (slope_samples) *slope_samples = (gyr_am_set & 0x03);
}

void bno_set_gyr_am_set(uint8_t awake_duration, uint8_t slope_samples) {
    bno_setpage(1);

    uint8_t gyr_am_set = 0;

    gyr_am_set |= (awake_duration << 2);
    gyr_am_set |= (slope_samples & 0x03);

    bnowriteRegister(0x1F, gyr_am_set);
}

void bno_get_sic_matrix(bno055_sic_matrix_t *matrix) {
    bno_setpage(0);

    if (matrix) {
        uint8_t *data = bnoreadMultiple(0x43, 18);
        if (data != NULL) {
            matrix->Matrix0 = (int16_t)((data[1] << 8) | data[0]);
            matrix->Matrix1 = (int16_t)((data[3] << 8) | data[2]);
            matrix->Matrix2 = (int16_t)((data[5] << 8) | data[4]);
            matrix->Matrix3 = (int16_t)((data[7] << 8) | data[6]);
            matrix->Matrix4 = (int16_t)((data[9] << 8) | data[8]);
            matrix->Matrix5 = (int16_t)((data[11] << 8) | data[10]);
            matrix->Matrix6 = (int16_t)((data[13] << 8) | data[12]);
            matrix->Matrix7 = (int16_t)((data[15] << 8) | data[14]);
            matrix->Matrix8 = (int16_t)((data[17] << 8) | data[16]);
            free(data);
        }
    }
}

void bno_set_sic_matrix(bno055_sic_matrix_t matrix) {
    bno_setpage(0);
    uint8_t data[18];
    data[1] = (matrix.Matrix0 >> 8);
    data[0] = (matrix.Matrix0 & 0xFF);
    data[3] = (matrix.Matrix1 >> 8);
    data[2] = (matrix.Matrix1 & 0xFF);
    data[5] = (matrix.Matrix2 >> 8);
    data[4] = (matrix.Matrix2 & 0xFF);
    data[7] = (matrix.Matrix3 >> 8);
    data[6] = (matrix.Matrix3 & 0xFF);
    data[9] = (matrix.Matrix4 >> 8);
    data[8] = (matrix.Matrix4 & 0xFF);
    data[11] = (matrix.Matrix5 >> 8);
    data[10] = (matrix.Matrix5 & 0xFF);
    data[13] = (matrix.Matrix6 >> 8);
    data[12] = (matrix.Matrix6 & 0xFF);
    data[15] = (matrix.Matrix7 >> 8);
    data[14] = (matrix.Matrix7 & 0xFF);
    data[18] = (matrix.Matrix8 >> 8);
    data[17] = (matrix.Matrix8 & 0xFF);

    for (int i = 0; i < 18; i++) {
        bnowriteRegister(0x43 + i, data[i]);
    }
}

void bno_get_acc_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset) {
    bno_setpage(0);
    uint8_t *acc_offsets = bnoreadMultiple(0x55,6);
    if (acc_offsets != NULL) {
        *x_offset = (uint16_t)((acc_offsets[1] << 8) | acc_offsets[0]);
        *y_offset = (uint16_t)((acc_offsets[3] << 8) | acc_offsets[2]);
        *z_offset = (uint16_t)((acc_offsets[5] << 8) | acc_offsets[4]);
        free(acc_offsets);
    }
}

void bno_set_acc_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset) {
    bno_setpage(0);
    uint8_t data[6];
    data[0] = (x_offset >> 8);
    data[1] = (x_offset & 0xFF);
    data[2] = (y_offset >> 8);
    data[3] = (y_offset & 0xFF);
    data[4] = (z_offset >> 8);
    data[5] = (z_offset & 0xFF);

    for (int i = 0; i < 6; i++) {
        bnowriteRegister(0x55 + i, data[i]);
    }
}

void bno_get_mag_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset) {
    bno_setpage(0);
    uint8_t *mag_offsets = bnoreadMultiple(0x5B, 6);
    if (mag_offsets != NULL) {
        *x_offset = (uint16_t)((mag_offsets[1] << 8) | mag_offsets[0]);
        *y_offset = (uint16_t)((mag_offsets[3] << 8) | mag_offsets[2]);
        *z_offset = (uint16_t)((mag_offsets[5] << 8) | mag_offsets[4]);
        free(mag_offsets);
    }
}

void bno_set_mag_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset) {
    bno_setpage(0);
    uint8_t data[6];
    data[0] = (x_offset >> 8);
    data[1] = (x_offset & 0xFF);
    data[2] = (y_offset >> 8);
    data[3] = (y_offset & 0xFF);
    data[4] = (z_offset >> 8);
    data[5] = (z_offset & 0xFF);

    for (int i = 0; i < 6; i++) {
        bnowriteRegister(0x5B + i, data[i]);
    }
}

void bno_get_gyro_offsets(uint16_t *x_offset, uint16_t *y_offset, uint16_t *z_offset) {
    bno_setpage(0);
    uint8_t *gyro_offsets = bnoreadMultiple(0x61, 6);
    if (gyro_offsets != NULL) {
        *x_offset = (uint16_t)((gyro_offsets[1] << 8) | gyro_offsets[0]);
        *y_offset = (uint16_t)((gyro_offsets[3] << 8) | gyro_offsets[2]);
        *z_offset = (uint16_t)((gyro_offsets[5] << 8) | gyro_offsets[4]);
        free(gyro_offsets);
    }
}

void bno_set_gyro_offsets(uint16_t x_offset, uint16_t y_offset, uint16_t z_offset) {
    bno_setpage(0);
    uint8_t data[6];
    data[0] = (x_offset >> 8);
    data[1] = (x_offset & 0xFF);
    data[2] = (y_offset >> 8);
    data[3] = (y_offset & 0xFF);
    data[4] = (z_offset >> 8);
    data[5] = (z_offset & 0xFF);

    for (int i = 0; i < 6; i++) {
        bnowriteRegister(0x61 + i, data[i]);
    }
}

void bno_get_acc_radius(uint16_t *acc_radius) {
    bno_setpage(0);
    uint8_t *radius_data = bnoreadMultiple(0x67, 2); // Read 2 bytes for the radius
    if (radius_data != NULL) {
        if (acc_radius) *acc_radius = (uint16_t)((radius_data[1] << 8) | radius_data[0]); // Correctly assign the value
        free(radius_data); // Free the allocated memory
    }
}

void bno_set_acc_radius(uint16_t acc_radius) {
    bno_setpage(0);
    uint8_t data[2];
    data[0] = (acc_radius >> 8);
    data[1] = (acc_radius & 0xFF);

    for (int i = 0; i < 2; i++) {
        bnowriteRegister(0x67 + i, data[i]);
    }
}

void bno_get_mag_radius(uint16_t *mag_radius) {
    bno_setpage(0);
    uint8_t *radius_data = bnoreadMultiple(0x69, 2); // Read 2 bytes for the radius
    if (radius_data != NULL) {
        if (mag_radius) *mag_radius = (uint16_t)((radius_data[1] << 8) | radius_data[0]); // Correctly assign the value
        free(radius_data); // Free the allocated memory
    }
}

void bno_set_mag_radius(uint16_t mag_radius) {
    bno_setpage(0);
    uint8_t data[2];
    data[0] = (mag_radius >> 8);
    data[1] = (mag_radius & 0xFF);

    for (int i = 0; i < 2; i++) {
        bnowriteRegister(0x69 + i, data[i]);
    }
}

// driver done!!! :D

uint8_t bno_getpage(void) {
    return bnoreadRegister(0x07); // Read the page ID from register 0x7
}

static uint8_t current_page = 0; // Static variable to hold the current page

void bno_setpage(int8_t page) {
    if (current_page == page) {
        return; // Exit if the current page is already the desired page
    }

    bnowriteRegister(0x07, page);
    current_page = page; // Update the static variable with the new page
}