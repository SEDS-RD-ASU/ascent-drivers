#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_h3lis331dl.h"
#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

static esp_err_t lisValidate(void) {
    printf("\n=== Current Sensor Configuration ===\n");
    fflush(stdout);
    esp_err_t ret = ESP_OK;

    // 1. Chip ID
    uint8_t chip_id;
    ret = h3lis331dl_get_chip_id(&chip_id);
    printf("Chip ID: 0x%02X\n", chip_id);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 2. CTRL_REG1 Settings
    h3lis331dl_axes_config_t axes_config;
    h3lis331dl_datarate_t datarate;
    h3lis331dl_power_mode_t power_mode;
    ret |= h3lis331dl_get_axes_config(&axes_config);
    ret |= h3lis331dl_get_datarate(&datarate);
    ret |= h3lis331dl_get_power_mode(&power_mode);
    
    printf("\nCTRL_REG1 Configuration:\n");
    printf("  - Axes Enabled: %s%s%s\n", 
           (axes_config & H3LIS331DL_AXIS_X) ? "X " : "",
           (axes_config & H3LIS331DL_AXIS_Y) ? "Y " : "",
           (axes_config & H3LIS331DL_AXIS_Z) ? "Z" : "");
    printf("  - Data Rate: %d Hz\n", 
           datarate == H3LIS331DL_DATARATE_50HZ ? 50 :
           datarate == H3LIS331DL_DATARATE_100HZ ? 100 :
           datarate == H3LIS331DL_DATARATE_400HZ ? 400 : 1000);
    printf("  - Power Mode: %s\n", 
           power_mode == H3LIS331DL_POWER_DOWN ? "Power Down" :
           power_mode == H3LIS331DL_NORMAL ? "Normal" : "Low Power");

    // 3. CTRL_REG2 Settings
    h3lis331dl_hpcf_t hpcf;
    h3lis331dl_hp_interrupt_t hp_int;
    h3lis331dl_filter_t filter;
    h3lis331dl_hpm_t hp_mode;
    ret |= h3lis331dl_get_hpcf(&hpcf);
    ret |= h3lis331dl_get_hp_interrupt(&hp_int);
    ret |= h3lis331dl_get_filter_mode(&filter);
    ret |= h3lis331dl_get_hp_mode(&hp_mode);

    printf("\nCTRL_REG2 Configuration:\n");
    printf("  - High-Pass Filter Cutoff: 0x%02X\n", hpcf);
    printf("  - HP Interrupt Mode: 0x%02X\n", hp_int);
    printf("  - Filter Mode: %s\n", filter == H3LIS331DL_FILTER_BYPASS ? "Bypass" : "Enabled");
    printf("  - HP Filter Mode: 0x%02X\n", hp_mode);

    // 4. CTRL_REG3 Settings (Interrupt Configuration)
    h3lis331dl_int1_data_t int1_cfg;
    h3lis331dl_int2_data_t int2_cfg;
    bool int1_latch, int2_latch;
    h3lis331dl_int_pin_mode_t pin_mode;
    h3lis331dl_int_level_t int_level;
    
    ret |= h3lis331dl_get_int1_config(&int1_cfg);
    ret |= h3lis331dl_get_int2_config(&int2_cfg);
    ret |= h3lis331dl_get_int1_latch(&int1_latch);
    ret |= h3lis331dl_get_int2_latch(&int2_latch);
    ret |= h3lis331dl_get_int_pin_mode(&pin_mode);
    ret |= h3lis331dl_get_int_level(&int_level);

    printf("\nInterrupt Configuration:\n");
    printf("  - INT1 Config: 0x%02X\n", int1_cfg);
    printf("  - INT2 Config: 0x%02X\n", int2_cfg);
    printf("  - INT1 Latch: %s\n", int1_latch ? "Enabled" : "Disabled");
    printf("  - INT2 Latch: %s\n", int2_latch ? "Enabled" : "Disabled");
    printf("  - Pin Mode: %s\n", pin_mode == H3LIS331DL_INT_PUSH_PULL ? "Push-Pull" : "Open-Drain");
    printf("  - Active Level: %s\n", int_level == H3LIS331DL_INT_ACTIVE_HIGH ? "High" : "Low");

    // 5. CTRL_REG4 Settings
    h3lis331dl_scale_t scale;
    h3lis331dl_endian_t endian;
    h3lis331dl_bdu_t bdu;
    
    ret |= h3lis331dl_get_scale(&scale);
    ret |= h3lis331dl_get_endian(&endian);
    ret |= h3lis331dl_get_bdu(&bdu);

    printf("\nScale and Data Configuration:\n");
    printf("  - Full Scale: ±%dg\n", 
           scale == H3LIS331DL_SCALE_100G ? 100 :
           scale == H3LIS331DL_SCALE_200G ? 200 : 400);
    printf("  - Data Format: %s\n", endian == H3LIS331DL_BIG_ENDIAN ? "Big Endian" : "Little Endian");
    printf("  - Block Data Update: %s\n", bdu == H3LIS331DL_BDU_CONTINUOUS ? "Continuous" : "Blocked");

    // 6. Status Register
    h3lis331dl_status_t status;
    ret |= h3lis331dl_get_status(&status);
    
    printf("\nSensor Status:\n");
    printf("  - New Data Available: %s%s%s\n",
           status.x_data_available ? "X " : "",
           status.y_data_available ? "Y " : "",
           status.z_data_available ? "Z" : "");
    printf("  - Data Overrun: %s%s%s\n",
           status.x_overrun ? "X " : "",
           status.y_overrun ? "Y " : "",
           status.z_overrun ? "Z" : "");

    printf("\n");
    fflush(stdout);
    return ret;
}

static esp_err_t lisRead(void) {
    fflush(stdout);
    
    esp_err_t ret = ESP_OK;
    double x_accel, y_accel, z_accel;
    
    ret = h3lis331dl_read_accel(&x_accel, &y_accel, &z_accel);
    if (ret == ESP_OK) {
        printf("Acceleration: X=%.2fg, Y=%.2fg, Z=%.2fg\n", x_accel, y_accel, z_accel);
    } else {
        printf("Failed to read accelerometer data (error: %d)\n", ret);
    }
    fflush(stdout);
    
    return ret;
}

void LIS331Test(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("\n\n=== H3LIS331DL Hardware Test ===\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize sensor
    printf("Initializing H3LIS331DL with SDA=%d, SCL=%d, Port=%d...\n", 
           I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_PORT);
    fflush(stdout);
    
    esp_err_t ret = h3lis331dl_init(I2C_MASTER_PORT);
    if (ret != ESP_OK) {
        printf("Failed to initialize H3LIS331DL (error: %d)\n", ret);
        fflush(stdout);
        return;
    }
    
    // Set power mode to normal
    ret = h3lis331dl_set_power_mode(H3LIS331DL_NORMAL);
    if (ret != ESP_OK) {
        printf("Failed to set power mode (error: %d)\n", ret);
        fflush(stdout);
        return;
    }
    
    printf("H3LIS331DL initialization successful\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Validate sensor configuration
    ret = lisValidate();
    if (ret != ESP_OK) {
        printf("Sensor validation failed (error: %d)\n", ret);
        return;
    }

    // Read sensor data
    printf("\nReading acceleration data:\n");
    //while(1) {
    for(int i = 0; i < 4; i++) {
        ret = lisRead();
        if (ret != ESP_OK) {
            printf("Sensor reading test failed (error: %d)\n", ret);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
