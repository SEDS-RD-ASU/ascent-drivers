#include <stdio.h>
#include <inttypes.h>
#include "driver_BMP390L.h"
#include <math.h>

static double groundPressure = 1013.25; // Default ground pressure in hPa

void bmp390_set_ground_pressure(double pressure) {
    groundPressure = pressure; // Set the ground pressure
}

// Required constants (define based on ISA)
#define SEA_LEVEL_PRESSURE 1013.25     // hPa
#define TEMP_LAPSE_RATE 0.0065         // K/m
#define GAS_CONSTANT 287.05            // J/(kg·K)
#define GRAVITY 9.80665                // m/s^2
#define METERS_TO_FEET 3.28084

uint32_t bmp390_barometricAGL(double pressure_hPa, double groundPressure_hPa) {
    if (pressure_hPa <= 0.0 || groundPressure_hPa <= 0.0) {
        printf("Invalid pressure input: %.2f / %.2f\n", pressure_hPa, groundPressure_hPa);
        return 0;
    }

    // ISA-based altitude (in meters)
    double alt_measured = 44330.0 * (1.0 - pow(pressure_hPa / 1013.25, 1.0 / 5.255));
    double alt_ground   = 44330.0 * (1.0 - pow(groundPressure_hPa / 1013.25, 1.0 / 5.255));

    double agl_meters = alt_measured - alt_ground;
    if (agl_meters < 0) agl_meters = 0;

    return (uint32_t)(agl_meters * METERS_TO_FEET + 0.5);  // return feet
}