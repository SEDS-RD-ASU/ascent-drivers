#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "driver_psu.h"

static const char* power_source_to_string(power_source_t source) {
    switch (source) {
        case POWER_SOURCE_BATTERY_ONLY:
            return "Battery Only";
        case POWER_SOURCE_USB_ONLY:
            return "USB Only";
        case POWER_SOURCE_BOTH:
            return "USB and Battery";
        case POWER_SOURCE_ERROR:
        default:
            return "Error/Unknown";
    }
}

static const char* battery_type_to_string(battery_type_t type) {
    switch (type) {
        case BATTERY_TYPE_1S_LITHIUM:
            return "1S Lithium";
        case BATTERY_TYPE_2S_LITHIUM:
            return "2S Lithium";
        case BATTERY_TYPE_9V:
            return "9V Battery";
        case BATTERY_TYPE_3S_LITHIUM:
            return "3S Lithium";
        case BATTERY_TYPE_LICB_12V:
            return "LiCB 12V";
        case BATTERY_TYPE_UNKNOWN:
            return "Unknown";
        case BATTERY_TYPE_NONE:
            return "No Battery";
        default:
            return "Invalid";
    }
}

static const char* psu_mode_to_string(psu_mode_t mode) {
    switch (mode) {
        case PSU_MODE_BUCK:
            return "Buck Mode";
        case PSU_MODE_BOOST:
            return "Boost Mode";
        case PSU_MODE_PERFECT_SOURCE:
            return "Perfect Source";
        case PSU_MODE_ERROR:
        default:
            return "Error";
    }
}

static esp_err_t psuValidate(void) {
    printf("\n=== Power Supply Status ===\n");
    fflush(stdout);
    esp_err_t ret = ESP_OK;

    // Read battery voltage
    double voltage = psu_read_battery_voltage();
    if (voltage < 0) {
        printf("Battery voltage reading failed\n");
        ret = ESP_FAIL;
    } else {
        printf("Battery voltage: %.3f V\n", voltage);
    }

    // Get power status
    power_status_t status = psu_get_power_source();
    printf("Power source: %s\n", power_source_to_string(status.source));
    
    // Print battery information if available
    if (status.source != POWER_SOURCE_USB_ONLY && status.source != POWER_SOURCE_ERROR) {
        printf("Battery type: %s\n", battery_type_to_string(status.battery_type));
        if (status.battery_percentage >= 0) {
            printf("Battery level: %d%%\n", status.battery_percentage);
        }
    }

    // Get and print PSU mode
    psu_mode_t mode = psu_get_mode();
    printf("PSU Mode: %s\n", psu_mode_to_string(mode));

    if (status.source == POWER_SOURCE_ERROR || mode == PSU_MODE_ERROR) {
        ret = ESP_FAIL;
    }

    fflush(stdout);
    return ret;
}

void PSUTest(void) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    printf("\n\n=== Power Supply Hardware Test ===\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize PSU with auto-detection
    printf("Initializing PSU...\n");
    fflush(stdout);
    
    esp_err_t ret = psu_init_default();  // or psu_init(BATTERY_TYPE_NONE);
    if (ret != ESP_OK) {
        printf("Failed to initialize PSU (error: %d)\n", ret);
        fflush(stdout);
        return;
    }
    printf("PSU initialization successful\n\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Continuous monitoring loop
    printf("Starting continuous monitoring (4 readings)...\n");
    for (int i = 0; i < 4; i++) {
        ret = psuValidate();
        if (ret != ESP_OK) {
            printf("PSU validation failed (error: %d)\n", ret);
            return;
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    printf("PSU test completed\n");
}
