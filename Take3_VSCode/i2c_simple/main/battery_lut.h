#ifndef BATTERY_LUT_H
#define BATTERY_LUT_H
#include "esp_log.h"

const uint16_t BATTERY_LUT[] = {
    3800,     ///3705,
    3780,     ///3696,
    3767,
    3761,
    3745,
    3737,
    3728,
    3715,
    3707,
    3689,
    3688,
    3667,
    3658,
    3646,
    3644,
    3634,
    3629,
    3621,
    3617,  //3622,
    3612,
    3560,  //3506,
    3526,
    3522,
    3513,
    3509,
    3499,
    3494,
    3489,
    3480,
    3477,
    3468,
    3462,
    3456,
    3452,
    3446,
    3445,
    3441,
    3434,
    3430,
    3425,
    3419,
    3416,
    3412, //3417,
    3410, //3408,
    3408, //3406,
    3406, //3407
    3404,
    3403, //3400,
    3401, //3402,
    3399, //3401,
    3397,
    3394,
    3386,
    3385,
    3379,
    3378,
    3367,
    3364,
    3359,
    3350,
    3341,
    3336,
    3327,
    3319,
    3312, //3305,
    3306,
    3296,
    3290,
    3285,
    3274,
    3268,
    3262,
    3255,
    3252,
    3244,
    3243,
    3240,
    3237,
    3228,
    3226,
    3222,
    3219,
    3213,
    3208,
    3197,
    3193,
    3181,
    3169,
    3157,
    3142,
    3117,
    3093,
    3069,
    3039,
    3009,
    2969,
    2924,
    2877,
    2824,
    2800 //16
};

void get_battery_percentage(uint16_t readAnalogValue) 
{
    for (uint8_t i = 0; i < 100; i++)
    {
        // keep iterating until we've gotten lower than the value in the LUT
        if (readAnalogValue <= BATTERY_LUT[i])
        {
            // LUT is backwards, so do a subtraction
            return 100 - i;
        }
    }

    return 100;
}


#endif