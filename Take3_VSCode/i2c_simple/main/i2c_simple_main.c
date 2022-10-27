#include <stdio.h>
#include <math.h>
#include <time.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "data_structures.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//static const char *TAG = "i2c-example";
#define I2C_SLAVE_ADDR	0x4A
#define TIMEOUT_MS		1000
#define DELAY_MS		1000
#define TIMER_DIVIDER   (64)

//Function Declarations
float qToFloat(uint16_t fixedPointValue, uint8_t qPoint);
void setup_BNO_I2C();
void setup_swing_Timer();
void printDP(struct DataPoint dp);
//void QuaternionCalculations();


float qToFloat(uint16_t fixedPointValue, uint8_t qPoint)
{
	short signedPointVal = fixedPointValue;
	float qFloat = signedPointVal * pow(2, (-1.0*qPoint));
	return qFloat;
}

void setup_BNO_I2C()
{
	uint8_t set_feature_linaccel[21]    = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x04, 0x0, 0x0, 0x0, 0xE8, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_gravity[21]     = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x06, 0x0, 0x0, 0x0, 0xE8, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_quaternion[21]  = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x05, 0x0, 0x0, 0x0, 0x20, 0x4E, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 22,
		.scl_io_num = 20,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 400000,
	};
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_set_timeout(I2C_NUM_0, 100000);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	//setting up FRC
	i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, set_feature_linaccel, 21, TIMEOUT_MS/portTICK_RATE_MS);
	i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, set_feature_gravity, 21, TIMEOUT_MS/portTICK_RATE_MS);
	i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, set_feature_quaternion, 21, TIMEOUT_MS/portTICK_RATE_MS);


	vTaskDelay(DELAY_MS/portTICK_RATE_MS);
}

void setup_swing_Timer()
{
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config); //initialize timer
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); //set timer initial value to 0 
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (TIMER_BASE_CLK / TIMER_DIVIDER));
    timer_enable_intr(TIMER_GROUP_0, TIMER_0); //enable timer interrupt
    //timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0); //start timer
}

void app_main() {
    //I2C
	uint8_t rx_data[23];
    setup_BNO_I2C();

    //Timer
    setup_swing_Timer();

    //Loop Variables
    int gotLinAccel = 0;
    int gotQuaternions = 0;
    int gotGravity = 0;

    float x, y, z, r, i, j, k;
    struct Quaternions quat;
    struct Gravity grav;
    struct LinearAcceleration linaccel;
    struct DataPoint dp;

    double timerSec;

	while (1) {
        i2c_master_read_from_device(I2C_NUM_0, I2C_SLAVE_ADDR, rx_data, 23, TIMEOUT_MS/portTICK_RATE_MS);

        if(rx_data[9] == 0x04) //Linear Acceleration
        {
            gotLinAccel = 1;

            uint8_t q = 8;
			uint16_t x_raw = (rx_data[14] << 8) | rx_data[13];
			uint16_t y_raw = (rx_data[16] << 8) | rx_data[15];
			uint16_t z_raw = (rx_data[18] << 8) | rx_data[17];
			x = qToFloat(x_raw, q);
			y = qToFloat(y_raw, q);
			z = qToFloat(z_raw, q);

            linaccel.x = x;
            linaccel.y = y;
            linaccel.z = z;
        }
        else if(rx_data[9] == 0x06) //Gravity
        {
            gotGravity = 1;

            uint8_t q = 8;
			uint16_t x_raw = (rx_data[14] << 8) | rx_data[13];
			uint16_t y_raw = (rx_data[16] << 8) | rx_data[15];
			uint16_t z_raw = (rx_data[18] << 8) | rx_data[17];
			x = qToFloat(x_raw, q);
			y = qToFloat(y_raw, q);
			z = qToFloat(z_raw, q);

            grav.x = x;
            grav.y = y;
            grav.z = z;
        }
        else if(rx_data[9] == 0x05) //Quaternion
        {
            gotQuaternions = 1;

            uint8_t q = 14;
            uint16_t r_raw = (rx_data[20] << 8) | rx_data[19];
            uint16_t i_raw = (rx_data[14] << 8) | rx_data[13];
            uint16_t j_raw = (rx_data[16] << 8) | rx_data[15];
            uint16_t k_raw = (rx_data[18] << 8) | rx_data[17];
            r = qToFloat(r_raw, q);
            i = qToFloat(i_raw, q);
            j = qToFloat(j_raw, q);
            k = qToFloat(k_raw, q);

            quat.r = r;
            quat.i = i;
            quat.j = j;
            quat.k = k;
        }

        if(gotLinAccel && gotGravity && gotQuaternions)
        {
            gotLinAccel = 0;
            gotGravity = 0;
            gotQuaternions = 0;

            dp.quat = quat;
            dp.grav = grav;
            dp.linaccel = linaccel;
            dp = dp; //GET RID OF
            //printDP(dp);

            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec);            
            //printf("%.3f\n", timerSec);
        }
	}

}

void printDP(struct DataPoint dp)
{
    printf("Lin Accel: (%.3f, %.3f, %.3f), ", dp.linaccel.x, dp.linaccel.y, dp.linaccel.y);
    printf("Grav: (%.3f, %.3f, %.3f), ", dp.grav.x, dp.grav.y, dp.grav.z);
    printf("Quat: (%.3f, %.3f, %.3f, %.3f)\n", dp.quat.r, dp.quat.i, dp.quat.j, dp.quat.k);
}