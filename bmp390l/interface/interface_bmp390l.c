#include <stdio.h>
#include <inttypes.h>
#include "driver_BMP390L.h"
#include <math.h>

static double groundPressure = 1013.25; // Default ground pressure in hPa

void set_ground_pressure(double pressure) {
    groundPressure = pressure; // Set the ground pressure
}

// Constants
#define SEA_LEVEL_PRESSURE 1013.25  // hPa
#define TEMP_LAPSE_RATE    -0.0065  // °C/m
#define GAS_CONSTANT       287.05   // J/(kg·K)
#define GRAVITY            9.80665  // m/s²
#define METERS_TO_FEET     3.28084

uint32_t barometricAGL(void) {
    double pressure, temperature;

    bmp390_read_sensor_data(&pressure, &temperature);

    printf("Read from sensor P and T: %f, %f\n", pressure, temperature);
    
    // Convert to Kelvin
    double temp_k = temperature + 273.15;

    // Compute pressure altitude (in meters)
    double pressure_alt = (temp_k / TEMP_LAPSE_RATE) * 
                          (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 
                                     (TEMP_LAPSE_RATE * GAS_CONSTANT) / -GRAVITY));

    // Altitude Above Ground Level (in meters)
    double agl_meters = pressure_alt - groundPressure;
    if (agl_meters < 0) agl_meters = 0;

    // Convert to feet
    double agl_feet = agl_meters * METERS_TO_FEET;

    return (uint32_t)(agl_feet + 0.5); // Rounded to nearest foot
}