#include "driver_pyro.h"
#include "ascent_r2_hardware_definition.h"
#include "adc_oneshot.h"
#include "esp_log.h"
#include "adc_cali.h"
#include "adc_cali_scheme.h"
#include "driver_psu.h"

static const char *TAG = "PYRO";
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc_cali_handle[4];  // One calibration handle per channel

// Array to map channel numbers to GPIO pins
static const gpio_num_t pyro_out_pins[] = {
    PYRO1_OUT, PYRO2_OUT, PYRO3_OUT, PYRO4_OUT
};

esp_err_t pyro_init(void) {
    // Configure output pins
    for (int i = 0; i < 4; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pyro_out_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        gpio_set_level(pyro_out_pins[i], 0);
    }

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &adc1_handle));

    // Initialize calibration for each channel
    for (int i = 0; i < 4; i++) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .chan = i,  // ADC_CHANNEL_0 through ADC_CHANNEL_3
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT
        };
        ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle[i]));
    }

    return ESP_OK;
}

bool pyro_continuity(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return false;
    }

    // Get the corresponding ADC channel from the enum
    adc_channel_t adc_chan;
    switch (channel) {
        case PYRO_CHANNEL_1:
            adc_chan = ADC_CHANNEL_0;
            break;
        case PYRO_CHANNEL_2:
            adc_chan = ADC_CHANNEL_1;
            break;
        case PYRO_CHANNEL_3:
            adc_chan = ADC_CHANNEL_2;
            break;
        case PYRO_CHANNEL_4:
            adc_chan = ADC_CHANNEL_3;
            break;
        default:
            return false;
    }

    // Configure ADC channel for this reading
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chan, &channel_config));

    // Read ADC
    int adc_raw;
    int voltage_mv;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_chan, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error on channel %d", channel);
        return false;
    }

    // Convert raw reading to millivolts using calibration
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle[adc_chan], adc_raw, &voltage_mv));
    double voltage = voltage_mv / 1000.0;
    
    // Return true if voltage is above 1V threshold
    return (voltage > 1.0);
}

double pyro_resistance(pyro_channel_t channel) {
    if (channel < 1 || channel > 4) {
        return -1.0;
    }

    // Check power source
    power_status_t power_status = psu_get_power_source();
    if (power_status.source != POWER_SOURCE_BOTH && power_status.source != POWER_SOURCE_BATTERY_ONLY) {
        ESP_LOGE(TAG, "Resistance can't be measured accurately with current power source.");
        return -1.0;
    }

    // Get the corresponding ADC channel from the enum
    adc_channel_t adc_chan;
    switch (channel) {
        case PYRO_CHANNEL_1:
            adc_chan = ADC_CHANNEL_0;
            break;
        case PYRO_CHANNEL_2:
            adc_chan = ADC_CHANNEL_1;
            break;
        case PYRO_CHANNEL_3:
            adc_chan = ADC_CHANNEL_2;
            break;
        case PYRO_CHANNEL_4:
            adc_chan = ADC_CHANNEL_3;
            break;
        default:
            return -1.0;
    }

    // Configure ADC channel for this reading
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_chan, &channel_config));

    // Read ADC
    int adc_raw;
    int voltage_mv;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_chan, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read error on channel %d", channel);
        return -1.0;
    }

    // Convert raw reading to millivolts using calibration
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle[adc_chan], adc_raw, &voltage_mv));
    double measured_voltage = voltage_mv / 1000.0;
    
    // Scale voltage based on voltage divider (multiply by 3.778)
    double actual_voltage = measured_voltage * DIVIDER_RATIO;

    // Calculate resistance
    double source_voltage = psu_read_battery_voltage();
    double effective_voltage = source_voltage - DIODE_DROP;
    double rematch = PYRO_RESISTANCE_COEFFICIENT * ((effective_voltage / actual_voltage) - DIVIDER_RATIO);

    if (rematch < 0) rematch = 0;

    ESP_LOGI(TAG, "Channel %d - Resistance: %.3f Ohms", channel, rematch);

    return rematch;
}

esp_err_t pyro_activate(pyro_channel_t channel, uint8_t delay) {
    if (channel < 1 || channel > 4) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check continuity before activation
    if (!pyro_continuity(channel)) {
        ESP_LOGE(TAG, "No continuity detected on channel %d", channel);
        return ESP_ERR_INVALID_STATE;
    }

    // Activate the pyro channel
    gpio_set_level(pyro_out_pins[channel - 1], 1);
    
    // You might want to add a delay here depending on your requirements
    vTaskDelay(pdMS_TO_TICKS(delay)); // Example: 100ms pulse

    // Reset the output
    gpio_set_level(pyro_out_pins[channel - 1], 0);

    return ESP_OK;
}

void pyro_deinit(void) {
    // Clean up calibration handles
    for (int i = 0; i < 4; i++) {
        adc_cali_delete_scheme_curve_fitting(adc_cali_handle[i]);
    }
    adc_oneshot_del_unit(adc1_handle);
}
