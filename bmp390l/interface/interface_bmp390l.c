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

static double groundPressure = 1013.25; // Default ground pressure in hPa

double altitude = 0.0;
double previous_altitude = 0.0;
double vertical_velocity = 0.0;
double vertical_acceleration = 0.0; // New global variable
double temperature = 0.0;

double pressure_value = 0.0; // Global variable to store pressure value

#define ALT_BUFFER_SIZE 60
#define VEL_BUFFER_SIZE 60 // New buffer size for velocity
typedef struct {
    double altitude;
    int64_t timestamp_us;
} alt_sample_t;

typedef struct {
    double velocity;
    int64_t timestamp_us;
} vel_sample_t; // New structure for velocity samples

typedef struct {
    double acceleration;
    int64_t timestamp_us;
} acc_sample_t;

#define ACC_BUFFER_SIZE 60  // Define the size of the buffer

acc_sample_t acc_buffer[ACC_BUFFER_SIZE];  // Declare the buffer for acceleration samples
int acc_index = 0;                         // Index for the current position in the acceleration buffer
bool acc_buffer_full = false;              // Flag to indicate if the buffer is full


alt_sample_t alt_buffer[ALT_BUFFER_SIZE];
vel_sample_t vel_buffer[VEL_BUFFER_SIZE]; // New velocity buffer
int alt_index = 0;
int vel_index = 0; // New index for velocity buffer
bool buffer_full = false;
bool vel_buffer_full = false; // New flag for velocity buffer


typedef struct {
    double velocity;
    double pressure;
    double altitude;
    double temperature;
    double acceleration; // New field for acceleration
} bmp390_vals_t;


// Function to get current sensor values
bmp390_vals_t bmp390_vals() {
    bmp390_vals_t vals;
    vals.velocity = vertical_velocity;
    vals.pressure = pressure_value;
    vals.altitude = altitude;
    vals.temperature = temperature;
    vals.acceleration = vertical_acceleration; // New line to include acceleration
    return vals;
}


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
    return num * 0.3048 / den; // ft/s
}

double calculate_acceleration() { // New function to calculate acceleration
    int n = vel_buffer_full ? VEL_BUFFER_SIZE : vel_index;
    if (n < 2) return 0.0;

    double sum_t = 0, sum_v = 0;
    for (int i = 0; i < n; i++) {
        sum_t += vel_buffer[i].timestamp_us / 1e6;
        sum_v += vel_buffer[i].velocity;
    }
    double mean_t = sum_t / n;
    double mean_v = sum_v / n;

    double num = 0, den = 0;
    for (int i = 0; i < n; i++) {
        double t = vel_buffer[i].timestamp_us / 1e6;
        double v = vel_buffer[i].velocity;
        num += (t - mean_t) * (v - mean_v);
        den += (t - mean_t) * (t - mean_t);
    }

    if (den == 0) return 0.0;
    return num * 0.3048 / den; 
}

void add_velocity_sample(double velocity, int64_t timestamp_us) { // New function to add velocity sample
    vel_buffer[vel_index].velocity = velocity;
    vel_buffer[vel_index].timestamp_us = timestamp_us;
    vel_index = (vel_index + 1) % VEL_BUFFER_SIZE;
    if (vel_index == 0) vel_buffer_full = true;
}

void add_sample(double altitude, int64_t timestamp_us) {
    alt_buffer[alt_index].altitude = altitude;
    alt_buffer[alt_index].timestamp_us = timestamp_us;
    alt_index = (alt_index + 1) % ALT_BUFFER_SIZE;
    if (alt_index == 0) buffer_full = true;
}

void add_acceleration_sample(double acceleration, int64_t timestamp_us) {
    acc_buffer[acc_index].acceleration = acceleration;
    acc_buffer[acc_index].timestamp_us = timestamp_us;
    acc_index = (acc_index + 1) % ACC_BUFFER_SIZE;
    if (acc_index == 0) acc_buffer_full = true;
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
        add_velocity_sample(vertical_velocity, now); // New line to add velocity sample
        vertical_acceleration = calculate_acceleration(); // New line to calculate acceleration
        add_acceleration_sample(vertical_acceleration, now);

        printf("Altitude: %.2f ft | Velocity: %.2f m/s | Accel: %.2f m/s²\n", altitude, vertical_velocity, vertical_acceleration); // Updated debug output

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}

void check_conditions_task(void *pvParameters) {
    const int sample_window = 100; // Number of samples in 10 seconds at 10 Hz
    double velocity_sum = 0.0, acceleration_sum = 0.0;
    int count = 0;

    while (1) {
        if (vel_buffer_full && buffer_full) {
            velocity_sum = 0.0;
            acceleration_sum = 0.0;
            count = 0;

            for (int i = 0; i < sample_window; i++) {
                int idx = (vel_index - i - 1 + VEL_BUFFER_SIZE) % VEL_BUFFER_SIZE;
                velocity_sum += vel_buffer[idx].velocity;
                acceleration_sum += acc_buffer[idx].acceleration;
                count++;
            }

            double avg_velocity = velocity_sum / count;
            double avg_acceleration = acceleration_sum / count;

            printf("Average Velocity: %.2f m/s, Average Acceleration: %.2f m/s²\n", avg_velocity, avg_acceleration);

            if (avg_velocity < 0 && avg_acceleration < 0 && altitude > 500) {
                note(NOTE_C, 6, 100);  // Higher pitch and shorter duration for a quick, loud beep
                vTaskDelay(100/portTICK_PERIOD_MS);
                note(NOTE_C, 6, 100);  // Repeat for emphasis
                printf("Dummy function triggered: Avg Velocity = %.2f, Avg Acceleration = %.2f\n", avg_velocity, avg_acceleration);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}