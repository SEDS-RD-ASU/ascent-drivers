#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_bno055.h"
#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

void BNOTest() {
    printf("\n=== BNO055 Full Test ===\n");
    fflush(stdout);
    
    // Initialize the BNO055 sensor
    esp_err_t ret = bno055_init(I2C_NUM_0);
    if (ret != ESP_OK) {
        printf("Failed to initialize BNO055: %d\n", ret);
        return;
    }

    printf("Resetting system...\n");

    bno_trigger_rst();

    // Set operation mode to NDOF
    bno_setoprmode(NDOF);
    printf("Operation mode set to NDOF.\n");

    // Read and print calibration status
    uint8_t sys, gyro, accel, mag;
    bno_get_calib(&sys, &gyro, &accel, &mag);
    printf("Calibration Status - Sys: %d, Gyro: %d, Accel: %d, Mag: %d\n", sys, gyro, accel, mag);

    // Read and print system status
    uint8_t sys_status = bno_getsysstatus();
    printf("System Status: %d\n", sys_status);

    // Read and print system error
    uint8_t sys_error = bno_getsyserror();
    printf("System Error: %d\n", sys_error);

    // Read and print clock status
    bool clock_status = bno_getclockstatus();
    printf("Clock Status: %s\n", clock_status ? "Configured State" : "Free to Configure");

    // Read and print units
    bool ori_unit, temp_unit, eul_unit, gyr_unit, acc_unit;
    bno_get_units(&ori_unit, &temp_unit, &eul_unit, &gyr_unit, &acc_unit);
    printf("Units - Orientation: %s, Temperature: %s, Euler: %s, Gyroscope: %s, Acceleration: %s\n",
       ori_unit ? "Android" : "Windows", 
       temp_unit ? "Fahrenheit" : "Celsius",
       eul_unit ? "Radians" : "Degrees",
       gyr_unit ? "Rps" : "Dps",
       acc_unit ? "mg" : "m/s^2");

    // Read and print temperature
    uint8_t temperature = bno_gettemp();
    printf("Temperature: %d °C\n", temperature);

    bno_setoprmode(AMG);
    printf("\n");

    printf("Reading BNO055 AMG data...\n");
    for(int i = 0; i < 10; i++) {
        int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;
        bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

        printf("Accel X: %d, Accel Y: %d, Accel Z: %d\n", acc_x, acc_y, acc_z);
        printf("Mag X: %d, Mag Y: %d, Mag Z: %d\n", mag_x, mag_y, mag_z);
        printf("Gyro X: %d, Gyro Y: %d, Gyro Z: %d\n", gyr_x, gyr_y, gyr_z);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");

    bno_setoprmode(NDOF);

    printf("Reading BNO055 EUL data...\n");
    for(int i = 0; i < 10; i++) {
        int16_t eul_heading, eul_roll, eul_pitch;
        bno_readeuler(&eul_heading, &eul_roll, &eul_pitch);

        printf("Heading: %d, Roll: %d, Pitch: %d\n", eul_heading, eul_roll, eul_pitch);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("\n");

    printf("Setting operation mode to config...\n");

    bno_setoprmode(CONFIG);

    printf("Setting power mode to normal...\n");

    bno_setpowermode(NORMAL);

    printf("Setting power mode to low power...\n");

    bno_setpowermode(LOWPOWER);

    printf("Setting power mode to suspend...\n");

    bno_setpowermode(SUSPEND);

    printf("Reverting power mode to normal...\n");

    bno_setpowermode(NORMAL);

    printf("\n");

    printf("Triggering self-test...\n");

    bno_trigger_st();

    vTaskDelay(450 / portTICK_PERIOD_MS); //needed 400ms delay

    // Read and print self-test results
    bool st_mcu, st_gyr, st_mag, st_acc;
    bno_getselftest(&st_mcu, &st_gyr, &st_mag, &st_acc);
    printf("Self-Test Results - MCU: %s, Gyro: %s, Mag: %s, Acc: %s\n",
           st_mcu ? "Passed" : "Failed", st_gyr ? "Passed" : "Failed",
           st_mag ? "Passed" : "Failed", st_acc ? "Passed" : "Failed");

    printf("\n");

    printf("Setting temperature source to accelerometer...\n");

    bno_set_tempsource(TEMP_ACC);

    printf("Setting temperature source to gyroscope...\n");

    bno_set_tempsource(TEMP_GYRO);

    printf("Reverting temperature source to accelerometer...\n");

    bno_set_tempsource(TEMP_ACC);  

    printf("\n");   

    printf("Triggering interrupt reset...\n");

    bno_trigger_int_rst();

    // Read and print interrupt status
    bool acc_nm, acc_am, acc_high_g, gyr_drdy, gyr_high_ratem, gyro_am, mag_drdy, acc_bsx_drdy;
    bno_getinterruptstatus(&acc_nm, &acc_am, &acc_high_g, &gyr_drdy, &gyr_high_ratem, &gyro_am, &mag_drdy, &acc_bsx_drdy);
    printf("Interrupts - Acc NM: %s, Acc AM: %s, Acc High G: %s, Gyro DRDY: %s, Gyro High Rate: %s, Gyro AM: %s, Mag DRDY: %s, Acc BSX DRDY: %s\n",
           acc_nm ? "Yes" : "No", acc_am ? "Yes" : "No", acc_high_g ? "Yes" : "No",
           gyr_drdy ? "Yes" : "No", gyr_high_ratem ? "Yes" : "No", gyro_am ? "Yes" : "No", mag_drdy ? "Yes" : "No", acc_bsx_drdy ? "Yes" : "No");
    
    printf("\n");

    printf("Changing to page 0...\n");

    bno_setpage(0);

    printf("Changing to page 1...\n");

    bno_setpage(1);

    printf("Reverting to page 0...\n");

    bno_setpage(0);

    printf("\nVerifying Interrupt Status and Mask...\n");

    // Get current interrupt status
    bno_getinterruptstatus(&acc_nm, &acc_am, &acc_high_g, &gyr_drdy, &gyr_high_ratem, &gyro_am, &mag_drdy, &acc_bsx_drdy);
    printf("Current Interrupt Status - Acc NM: %s, Acc AM: %s, Acc High G: %s, Gyro DRDY: %s, Gyro High Rate: %s, Gyro AM: %s, Mag DRDY: %s, Acc BSX DRDY: %s\n",
           acc_nm ? "Yes" : "No", acc_am ? "Yes" : "No", acc_high_g ? "Yes" : "No",
           gyr_drdy ? "Yes" : "No", gyr_high_ratem ? "Yes" : "No", gyro_am ? "Yes" : "No", mag_drdy ? "Yes" : "No", acc_bsx_drdy ? "Yes" : "No");

    // Set interrupt mask
    bno_setinterruptmask(true, true, true, true, true, true, true, true);
    printf("Interrupt Mask Set to Enable All.\n");

    // Verify interrupt mask
    bno_getinterruptmask(&acc_nm, &acc_am, &acc_high_g, &gyr_drdy, &gyr_high_ratem, &gyro_am, &mag_drdy, &acc_bsx_drdy);
    printf("New Interrupt Mask - Acc NM: %s, Acc AM: %s, Acc High G: %s, Gyro DRDY: %s, Gyro High Rate: %s, Gyro AM: %s, Mag DRDY: %s, Acc BSX DRDY: %s\n",
           acc_nm ? "Enabled" : "Disabled", acc_am ? "Enabled" : "Disabled", acc_high_g ? "Enabled" : "Disabled",
           gyr_drdy ? "Enabled" : "Disabled", gyr_high_ratem ? "Enabled" : "Disabled", gyro_am ? "Enabled" : "Disabled", mag_drdy ? "Enabled" : "Disabled", acc_bsx_drdy ? "Enabled" : "Disabled");

    // Set interrupt mask
    bno_setinterruptmask(false, false, false, false, false, false, false, false);
    printf("Interrupt Mask set to disable all.\n");

    printf("\n");

    printf("Resetting system...\n");

    bno_trigger_rst();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    printf("\n");

    printf("\nAxis Map Configuration Test...\n");

    bno_axismap axis_map = bno_get_axismapconfig();
    printf("Current Axis Map Configuration - X: %d, Y: %d, Z: %d\n", axis_map.x, axis_map.y, axis_map.z);

    bno_axismap modified = {1,2,0};
    bno_axismap normal = {0,1,2};
    bno_set_axismapconfig(modified);

    bno_axismap newaxis_map = bno_get_axismapconfig();
    printf("New Axis Map Configuration - X: %d, Y: %d, Z: %d\n", newaxis_map.x, newaxis_map.y, newaxis_map.z);

    bno_set_axismapconfig(normal);
    bno_axismap normalaxis_map = bno_get_axismapconfig();
    printf("Reverted to normal Axis Map Configuration - X: %d, Y: %d, Z: %d\n", normalaxis_map.x, normalaxis_map.y, normalaxis_map.z);

    printf("\n");

    printf("Resetting system...\n");

    bno_trigger_rst();

    bno_setoprmode(CONFIG);

    //attempt to set to defaults

    printf("\nConfiguring Accelerometer, Magnetometer, and Gyroscope to defaults...\n");
    bno_configure_acc(ACC_C_NORMAL, ACC_C_H62_50,ACC_C_RANGE_4G);
    bno_config_mag(MAG_C_NORMAL, MAG_C_REGULAR, MAG_C_H10);
    bno_configure_gyro(GYRO_C_D2000, GYRO_C_H32, GYRO_C_NORMAL);

    printf("\nReading config from registers:\n");
    bno055_acc_pwrmode_t acc_pwr;
    bno055_acc_bandwidth_t acc_bandwidth;
    bno055_acc_range_t acc_range;
    bno_getacc_config(&acc_pwr, &acc_bandwidth, &acc_range);
    printf("Accelerometer Configuration - Power Mode: %d, Bandwidth: %d, Range: %d\n", acc_pwr, acc_bandwidth, acc_range);

    bno055_mag_pwrmode_t mag_pwr;
    bno055_mag_oprmode_t mag_oprmode;
    bno055_mag_datarate_t mag_datarate;
    bno_getmag_config(&mag_pwr, &mag_oprmode, &mag_datarate);
    printf("Magnetometer Configuration - Power Mode: %d, Operation Mode: %d, Data Rate: %d\n", mag_pwr, mag_oprmode, mag_datarate);

    bno055_gyro_range_t gyro_range;
    bno055_gyro_bandwidth_t gyro_bandwidth;
    bno055_gyro_powermode_t gyro_pwr;
    bno_getgyro_config(&gyro_range, &gyro_bandwidth, &gyro_pwr);
    printf("Gyroscope Configuration - Range: %d, Bandwidth: %d, Power Mode: %d\n", gyro_range, gyro_bandwidth, gyro_pwr);

    //attempt to max out the config

    printf("\nConfiguring Accelerometer, Magnetometer, and Gyroscope to max...\n");
    bno_configure_acc(ACC_C_NORMAL, ACC_C_H1000, ACC_C_RANGE_16G);
    bno_config_mag(MAG_C_NORMAL, MAG_C_HIGHACCURACY, MAG_C_H30);
    bno_configure_gyro(GYRO_C_D2000, GYRO_C_H523, GYRO_C_NORMAL);

    printf("\nReading config from registers:\n");
    bno_getacc_config(&acc_pwr, &acc_bandwidth, &acc_range);
    printf("Accelerometer Configuration - Power Mode: %d, Bandwidth: %d, Range: %d\n", acc_pwr, acc_bandwidth, acc_range);

    bno_getmag_config(&mag_pwr, &mag_oprmode, &mag_datarate);
    printf("Magnetometer Configuration - Power Mode: %d, Operation Mode: %d, Data Rate: %d\n", mag_pwr, mag_oprmode, mag_datarate);

    bno_getgyro_config(&gyro_range, &gyro_bandwidth, &gyro_pwr);
    printf("Gyroscope Configuration - Range: %d, Bandwidth: %d, Power Mode: %d\n", gyro_range, gyro_bandwidth, gyro_pwr);

    //revert to defaults

    printf("\nConfiguring Accelerometer, Magnetometer, and Gyroscope to defaults...\n");
    bno_configure_acc(ACC_C_NORMAL, ACC_C_H62_50, ACC_C_RANGE_4G);
    bno_config_mag(MAG_C_NORMAL, MAG_C_REGULAR, MAG_C_H10);
    bno_configure_gyro(GYRO_C_D2000, GYRO_C_H32, GYRO_C_NORMAL);

    printf("\nReading config from registers:\n");
    bno_getacc_config(&acc_pwr, &acc_bandwidth, &acc_range);
    printf("Accelerometer Configuration - Power Mode: %d, Bandwidth: %d, Range: %d\n", acc_pwr, acc_bandwidth, acc_range);

    bno_getmag_config(&mag_pwr, &mag_oprmode, &mag_datarate);
    printf("Magnetometer Configuration - Power Mode: %d, Operation Mode: %d, Data Rate: %d\n", mag_pwr, mag_oprmode, mag_datarate);

    bno_getgyro_config(&gyro_range, &gyro_bandwidth, &gyro_pwr);
    printf("Gyroscope Configuration - Range: %d, Bandwidth: %d, Power Mode: %d\n", gyro_range, gyro_bandwidth, gyro_pwr);

    printf("\nResetting system...\n");

    bno_trigger_rst();

    printf("\n\nBNO055 Full Test Completed.\n");
}

void spit_out_data() {
    fflush(stdout);
    bno_setoprmode(NDOF);

    while(1) {
    int16_t acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z;
    bno_readamg(&acc_x, &acc_y, &acc_z, &mag_x, &mag_y, &mag_z, &gyr_x, &gyr_y, &gyr_z);

    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", acc_x, acc_y, acc_z, mag_x, mag_y, mag_z, gyr_x, gyr_y, gyr_z);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}