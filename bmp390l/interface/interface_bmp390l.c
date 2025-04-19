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

#define NUM_READINGS 30 // Define the number of readings to take


void update_ground_pressure() {

    buzzer_init();

    double totalPressure = 0.0; // Initialize total pressure

    // Loop over the number of readings
    for (int i = 0; i < NUM_READINGS; i++) {
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
    groundPressure = totalPressure / NUM_READINGS;
    printf("Ground pressure updated: %.2f hPa\n", groundPressure);
    note(NOTE_E,6,100);
}

// Required constants (define based on ISA)
#define SEA_LEVEL_PRESSURE 1013.25     // hPa
#define TEMP_LAPSE_RATE 0.0065         // K/m
#define GAS_CONSTANT 287.05            // J/(kg·K)
#define GRAVITY 9.80665                // m/s^2

float bmp390_barometricAGL() {

    double pressure_hPa;
    double temperature;

    bmp390_read_sensor_data(&pressure_hPa, &temperature);

    if (pressure_hPa <= 0.0 || groundPressure <= 0.0) {
        printf("Invalid pressure input: %.2f / %.2f\n", pressure_hPa, groundPressure);
        return 0;
    }

    // ISA-based altitude (in meters)
    double alt_measured = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 1.0 / 5.255));
    double alt_ground   = 44330.0 * (1.0 - pow(groundPressure / 1013.25, 1.0 / 5.255));

    double agl_meters = alt_measured - alt_ground;
    if (agl_meters < 0) agl_meters = 0;

    return agl_meters;  // return feet as a float
}