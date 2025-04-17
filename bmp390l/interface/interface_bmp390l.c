#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "driver_BMP390L.h"
#include "math.h"
#include "esp_timer.h"


static double groundPressure = 1013.25; // Default ground pressure in hPa

double altitude = 0.0;
double previous_altitude = 0.0;
double vertical_velocity = 0.0;
double temperature = 0.0;

double pressure_value = 0.0; // Global variable to store pressure value

#define ALT_BUFFER_SIZE 5

typedef struct {
    double altitude;
    int64_t timestamp_us;
} alt_sample_t;

alt_sample_t alt_buffer[ALT_BUFFER_SIZE];
int alt_index = 0;
bool buffer_full = false;


void update_ground_pressure() {
    double totalPressure = 0.0;
    for (int i = 0; i < ALT_BUFFER_SIZE; i++) {
        double pressure, temperature;
        esp_err_t ret = bmp390_read_sensor_data(&pressure, &temperature);
        if (ret != ESP_OK) {
            ESP_LOGE("BMP390L", "Failed to read sensor data");
            return;
        }
        totalPressure += pressure;
        vTaskDelay(pdMS_TO_TICKS(150)); // 150 ms delay
    }

    groundPressure = totalPressure / ALT_BUFFER_SIZE;
}

// Required constants (define based on ISA)
#define SEA_LEVEL_PRESSURE 1013.25     // hPa
#define TEMP_LAPSE_RATE 0.0065         // K/m
#define GAS_CONSTANT 287.05            // J/(kg·K)
#define GRAVITY 9.80665                // m/s^2
#define METERS_TO_FEET 3.28084

float bmp390_barometricAGL(double pressure_hPa, double groundPressure_hPa) {
    if (pressure_hPa <= 0.0 || groundPressure_hPa <= 0.0) {
        printf("Invalid pressure input: %.2f / %.2f\n", pressure_hPa, groundPressure_hPa);
        return 0;
    }

    // ISA-based altitude (in meters)
    double alt_measured = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 1.0 / 5.255));
    double alt_ground   = 44330.0 * (1.0 - pow(groundPressure_hPa / 1013.25, 1.0 / 5.255));

    double agl_meters = alt_measured - alt_ground;
    if (agl_meters < 0) agl_meters = 0;

    return agl_meters * METERS_TO_FEET;  // return feet as a float
}

double calculate_velocity() {
    int n = buffer_full ? ALT_BUFFER_SIZE : alt_index;
    if (n < 2) return 0.0;

    // Compute means
    double sum_t = 0, sum_h = 0;
    for (int i = 0; i < n; i++) {
        sum_t += alt_buffer[i].timestamp_us / 1e6;
        sum_h += alt_buffer[i].altitude;
    }
    double mean_t = sum_t / n;
    double mean_h = sum_h / n;

    // Compute slope (velocity)
    double num = 0, den = 0;
    for (int i = 0; i < n; i++) {
        double t = alt_buffer[i].timestamp_us / 1e6;
        double h = alt_buffer[i].altitude;
        num += (t - mean_t) * (h - mean_h);
        den += (t - mean_t) * (t - mean_t);
    }

    if (den == 0) return 0.0;
    return num / den; // m/s
}

void add_sample(double altitude, int64_t timestamp_us) {
    alt_buffer[alt_index].altitude = altitude;
    alt_buffer[alt_index].timestamp_us = timestamp_us;
    alt_index = (alt_index + 1) % ALT_BUFFER_SIZE;
    if (alt_index == 0) buffer_full = true;
}

void update_vals(void *pvParameters) {
    // Assuming pvParameters is not used in this task
    (void)pvParameters;

    while (1) { // Task should run indefinitely
        double temperature;
        int64_t now = esp_timer_get_time();

        bmp390_read_sensor_data(&pressure_value, &temperature);
        altitude = bmp390_barometricAGL(pressure_value, groundPressure); // Corrected variable name

        add_sample(altitude, now);
        vertical_velocity = calculate_velocity();

        printf("Altitude: %.2f m | Velocity: %.2f m/s\n", altitude, vertical_velocity);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

typedef struct {
    double velocity;
    double pressure;
    double altitude;
    double temperature;
} bmp390_vals_t;

// Function to get current sensor values
bmp390_vals_t bmp390_vals() {
    bmp390_vals_t vals;
    vals.velocity = vertical_velocity;
    vals.pressure = pressure_value;
    vals.altitude = altitude;
    vals.temperature = temperature;
    return vals;
}