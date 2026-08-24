#include "voltageReader.h"

#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_adc/adc_oneshot.h" // “One-shot” means the ADC takes one measurement whenever the program asks for one.
#include "esp_err.h" // ESP Error Handling Library

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VOLTAGE_READER_ADC_UNIT ADC_UNIT_2
#define VOLTAGE_READER_ADC_CHANNEL ADC_CHANNEL_2 // On the ESP32-S3, GPIO 13 is connected to ADC2 channel 2.
#define VOLTAGE_DIVIDER_RATIO 5.0f
#define BATTERY_FULL_VOLTAGE 12.6f
#define BATTERY_WARNING_VOLTAGE 11.1f
#define BATTERY_CUTOFF_VOLTAGE 10.5f
    
static adc_oneshot_unit_handle_t voltageReaderAdcHandle = NULL; // Handle for the ADC unit used for voltage reading
static adc_cali_handle_t voltageReaderCalibrationHandle = NULL;
static void voltageReaderTask(void *arg);
static float voltageReaderGetPercentage(float batteryVoltage);

void voltageReaderInit(void)
{
    // ------------------------- ADC Unit Configuration --------------------------
    adc_oneshot_unit_init_cfg_t unitConfig = {
        .unit_id = VOLTAGE_READER_ADC_UNIT, // ADC_UNIT_2
        .ulp_mode = ADC_ULP_MODE_DISABLE // ulp = ultra-low-power
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit( // initializes the ADC unit
        &unitConfig, &voltageReaderAdcHandle // applying the configuration to the handle
    )); // ESP_ERROR_CHECK checks if the function call was successful
        

    // ------------------------- ADC Channel Configuration -----------------------
    adc_oneshot_chan_cfg_t channelConfig = {
        .atten = ADC_ATTEN_DB_12, // attenuation = lowering the input voltage internally to read higher voltages
        .bitwidth = ADC_BITWIDTH_DEFAULT // selecting the maximum supported bit width as the default bit width
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        voltageReaderAdcHandle, // Handle
        VOLTAGE_READER_ADC_CHANNEL, // ADC_CHANNEL_2
        &channelConfig // Configuration
    )); // "Configure the channel of the handle with the specified channel configuration"

    // ------------------------- ADC Calibration Configuration -------------------
    adc_cali_curve_fitting_config_t calibrationConfig = {
        .unit_id = VOLTAGE_READER_ADC_UNIT,
        .chan = VOLTAGE_READER_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting( // creates a calibration scheme for specific ADC unit and channel
        &calibrationConfig,
        &voltageReaderCalibrationHandle // this is not an actual read value. It's the calibraion ratio
    ));

    // ------------------------- Create Voltage Reader Task -----------------------
    xTaskCreate(voltageReaderTask, "voltageReaderTask", 4096, NULL, 1, NULL);

}
float voltageReaderRead(void)
{
    int rawValue;
    ESP_ERROR_CHECK(adc_oneshot_read(
        voltageReaderAdcHandle,
        VOLTAGE_READER_ADC_CHANNEL,
        &rawValue
    ));

    int gpioMillivolts;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
        voltageReaderCalibrationHandle,
        rawValue,
        &gpioMillivolts
    ));

    return ((float)gpioMillivolts / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
}

static float voltageReaderGetPercentage(float batteryVoltage)
{
    float percentage =
        ((batteryVoltage - BATTERY_CUTOFF_VOLTAGE) /
         (BATTERY_FULL_VOLTAGE - BATTERY_CUTOFF_VOLTAGE)) * 100.0f;

    if (percentage < 0.0f) {
        return 0.0f;
    }

    if (percentage > 100.0f) {
        return 100.0f;
    }

    return percentage;
}

static void voltageReaderTask(void *arg)
{
    (void)arg;

    while (1) {
        float batteryVoltage = voltageReaderRead();
        float batteryPercentage = voltageReaderGetPercentage(batteryVoltage);

        if (batteryVoltage <= BATTERY_CUTOFF_VOLTAGE) {
            printf("Battery CUTOFF: %.0f%% (%.2f V)\n", batteryPercentage, batteryVoltage);
        }
        else if (batteryVoltage <= BATTERY_WARNING_VOLTAGE) {
            printf("Battery LOW: %.0f%% (%.2f V)\n", batteryPercentage, batteryVoltage);
        }
        else {
            printf("Battery: %.0f%% (%.2f V)\n", batteryPercentage, batteryVoltage);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
