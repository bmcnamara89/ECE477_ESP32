#include "battery_lut.h"


void config_battery_read_pin()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN_DB_11); // change ADC_CHANNEL_3 to ADC_CHANNEL_7 when on PCB 

    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

uint8_t get_battery_percentage() 
{
    uint32_t readAnalogValue = 0;

    for (uint8_t i = 0; i < SAMPLES_TO_TAKE; i++) 
    {
        readAnalogValue += adc1_get_raw((adc1_channel_t)ADC_CHANNEL_7); // change ADC_CHANNEL_3 to ADC_CHANNEL_7 when on PCB
    }

    // take the total read value and divide by the number of samples we took
    readAnalogValue /= SAMPLES_TO_TAKE;

    // special case for if the battery is being charged
    if (readAnalogValue >= 4090) {
        return 101;
    }

    for (uint8_t i = 0; i < 100; i++)
    {
        // keep iterating until we've gotten higher than the value in the LUT
        if (readAnalogValue >= BATTERY_LUT[i])
        {
            // LUT is backwards, so do a subtraction
            return 100 - i;
        }
    }

    return 0;
}