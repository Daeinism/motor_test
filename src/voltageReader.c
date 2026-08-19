#include "voltageReader.h"

#include <stdio.h>

#include "esp_adc/adc_oneshot.h" // “One-shot” means the ADC takes one measurement whenever the program asks for one.
#include "esp_err.h" // ESP Error Handling Library

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VOLTAGE_READER_ADC_UNIT ADC_UNIT_2
#define VOLTAGE_READER_ADC_CHANNEL ADC_CHANNEL_2 // On the ESP32-S3, GPIO 13 is connected to ADC2 channel 2.
#define ADC_MAX_RAW_VALUE 4095.0f // Default maximum raw value for 12-bit ADC resolution (2^12 - 1 = 4095)
#define ESP32_REFERENCE_VOLTAGE 3.3f
#define VOLTAGE_DIVIDER_RATIO 5.0f
    

static adc_oneshot_unit_handle_t voltageReaderAdcHandle = NULL; // Handle for the ADC unit used for voltage reading
static void voltageReaderTask(void *arg);

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

    float gpioVoltage = ((float)rawValue / ADC_MAX_RAW_VALUE) * ESP32_REFERENCE_VOLTAGE;
    return gpioVoltage * VOLTAGE_DIVIDER_RATIO;
}
static void voltageReaderTask(void *arg)
{
    (void)arg;

    while (1) {
        printf("Battery voltage: %.2f V\n", voltageReaderRead());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
