#include "battery_lut.h"


void config_battery_read_pin()
{
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

uint8_t get_battery_percentage() 
{
    uint32_t readAnalogValue = 0;

    for (uint8_t i = 0; i < SAMPLES_TO_TAKE; i++) 
    {
        readAnalogValue += adc1_get_raw((adc1_channel_t)channel);
    }
    readAnalogValue /= SAMPLES_TO_TAKE;

    printf("RAW: %d", readAnalogValue);


    for (uint8_t i = 0; i < 100; i++)
    {
        // keep iterating until we've gotten lower than the value in the LUT
        if (readAnalogValue >= BATTERY_LUT[i])
        {
            // LUT is backwards, so do a subtraction
            return 100 - i;
        }
    }

    return 0;
}