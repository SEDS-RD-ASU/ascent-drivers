#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "driver_BMP390L.h"
#include "math.h"
#include "driver_buzzer.h"
#include "ascent_r2_hardware_definition.h"

void bmp390_sensorinit() {
    bmp390_init(I2C_MASTER_PORT);

    bmp390_osr_settings_t osr_settings = {
        .press_os = BMP390_OVERSAMPLING_2X,
        .temp_os = BMP390_OVERSAMPLING_2X
    };

    bmp390_set_osr(&osr_settings);

    bmp390_odr_t odr_settings = BMP390_ODR_100HZ;

    bmp390_set_odr(odr_settings);

    bmp390_config_t filterconfig = {
        .iir_filter = BMP390_IIR_FILTER_COEFF_63
    };

    bmp390_set_config(&filterconfig);

    printf("BMP Configured!\n");
}

static double groundPressure = 1013.25; // Default ground pressure in hPa

void update_ground_pressure(double *groundPressure, uint8_t num_readings) {
    groundPressure = 1013.25;

    double totalPressure = 0.0; // Initialize total pressure

    // Loop over the number of readings
    for (int i = 0; i < num_readings; i++) {
        double pressure, temperature; // Declare pressure and temperature variables

        // Read sensor data and store the return value
        esp_err_t ret = bmp390_read_sensor_data(&pressure, &temperature);

        // If the sensor data read is not successful, log an error and return
        if (ret != ESP_OK) {
            ESP_LOGE("BMP390L", "Failed to read sensor data");
            return;
        }

        totalPressure += pressure; // Add the pressure to the total pressure

        vTaskDelay(pdMS_TO_TICKS(30)); // Delay for 150 ms
    }

    // Calculate the average ground pressure
    groundPressure = totalPressure / num_readings;
}

void pressure_to_m(double *pressure, double *temperature, double *alt) {
    if (pressure <= 0.0) {
        printf("Invalid pressure input: %.2f\n", pressure);
        return 0;
    }

    *alt = ((*temperature+273.15)/0.0065) * (1.0 - pow(*pressure / 1013.25, 1.0 / 5.255));
}