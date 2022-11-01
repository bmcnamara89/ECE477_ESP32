#include <stdio.h>
#include <math.h>
#include <time.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "data_structures.h"
#include "bluetooth.h"
//#include "sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//static const char *TAG = "i2c-example";
#define I2C_SLAVE_ADDR	0x4A
#define TIMEOUT_MS		1000
#define DELAY_MS		1000
#define TIMER_DIVIDER   (64)
#define ACCEL_THRESHOLD 10.0

//Function Declarations
float qToFloat(uint16_t fixedPointValue, uint8_t qPoint);
void setup_BNO_I2C();
void setup_swing_Timer();
void printDP(struct DataPoint dp);
float getMagnitude(float x, float y, float z);
void printDPS(struct DataPoint *dps, int numDPs);
void fakeReckoning(struct DataPoint *dps, int numDPs);
void print_buffer(struct DataOut* data, uint16_t len);
void deadReckoning(struct DataPoint *dps, int numDPs);
void pointReckoning(struct DataPoint dp);
void firstPointReckoning(struct DataPoint dp);

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

    //Bluetooth
    init_ble();

    //Loop Variables
    int gotLinAccel = 0;
    int gotQuaternions = 0;
    int gotGravity = 0;

    float x, y, z, r, i, j, k;
    struct Quaternions quat;
    struct Coordinates grav;
    struct Coordinates linaccel;
    struct DataPoint dp;

    double timerSec;

    //Swing Detection
    int swingNum = 0;
    int inSwing = 0;
    double swingStartTime = 0.0;

    struct DataPoint *dps = NULL;
    int dpnum = 0;

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

            if(inSwing == 0 && getMagnitude(x, y, z) > ACCEL_THRESHOLD) //Start of Swing Detected
            {
                inSwing = 1;
                swingNum++;
                timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
                //timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec); 
                swingStartTime = 0.0; //timerSec;

                gotGravity = 0;
                gotQuaternions = 0;
                
                dps = (struct DataPoint *) malloc(sizeof(struct DataPoint) * 50);
            }
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
            timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec); 
            dp.time = timerSec;
            
            //printDP(dp);

            if(inSwing == 1)
            {
                dps[dpnum] = dp;
                dpnum++;
                //printf("Swing %d: from %.3f to %.3f of Length %.3f seconds\n", swingNum, swingStartTime, timerSec, timerSec - swingStartTime);
            }
        }
        
        timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec);
        if(inSwing == 1 && timerSec- swingStartTime > 0.4) //END Swing 
        {
            inSwing = 0;

            fakeReckoning(dps, dpnum);
            //deadReckoning(dps, dpnum); //store in outputdatapoints

            free(dps);
            dpnum = 0; 
        }
    
	}

}

void printDP(struct DataPoint dp)
{
    printf("Lin Accel: (%.3f, %.3f, %.3f), ", dp.linaccel.x, dp.linaccel.y, dp.linaccel.y);
    printf("Grav: (%.3f, %.3f, %.3f), ", dp.grav.x, dp.grav.y, dp.grav.z);
    printf("Quat: (%.3f, %.3f, %.3f, %.3f)\n", dp.quat.r, dp.quat.i, dp.quat.j, dp.quat.k);
}

float getMagnitude(float x, float y, float z)
{
    return abs(pow(x*x + y*y + z*z, 0.5));
}

void printDPS(struct DataPoint *dps, int numDPs)
{   
    printf("[");
    for(int i = 0; i < numDPs; i++)
    {
        printf("%d:%.4f | ", i+1, dps[i].time);
    }
    printf("]\n");
}

void fakeReckoning(struct DataPoint *dps, int numDPs)
{ 
    int tempNum = 10;
    struct DataOut * outputData;  
    outputData = (struct DataOut *) malloc(sizeof(struct DataOut) * tempNum); //switch tempNUm for numDPs

    //switched numDPs to tempNum for testing
    for(int i = 0; i < tempNum; i++)
    {
        outputData[i].pos.x = 0.362;
        outputData[i].pos.y = 4.77;
        outputData[i].pos.z = 20.001;
        outputData[i].quat.r = dps[i].quat.r;
        outputData[i].quat.i = dps[i].quat.i;
        outputData[i].quat.j = dps[i].quat.j;
        outputData[i].quat.k = dps[i].quat.k;
        outputData[i].time = dps[i].time;
    }

    printf("Buffer Filled: %d Data Points\n", tempNum);
    print_buffer(outputData, tempNum);
    set_transmit_buffer(outputData, tempNum);

}

void print_buffer(struct DataOut* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) 
    {
        printf("Position: <%f, %f, %f>    Quaternion: <%f, %f, %f, %f>    Time: %lf\n", data[i].pos.x, data[i].pos.y, data[i].pos.z, data[i].quat.r, data[i].quat.i, data[i].quat.j, data[i].quat.k, data[i].time);
    }
}

void deadReckoning(struct DataPoint *dps, int numDPs)
{ 
    struct DataOut * outputData;  
    outputData = (struct DataOut *) malloc(sizeof(struct DataOut) * numDPs); //switch tempNUm for numDPs

    //switched numDPs to tempNum for testing
    for(int i = 0; i < numDPs; i++)
    {
        //Call Dead Reckoning on Each Point
        if(i > 0)
        {
            pointReckoning(dps[i]);
        }
        else
        {
            firstPointReckoning(dps[i]);
        }
    }

    printf("Buffer Filled: %d Data Points\n", numDPs);
    print_buffer(outputData, numDPs);
    set_transmit_buffer(outputData, numDPs);

}

void pointReckoning(struct DataPoint dp)
{
    //ConvertQuaternionToRotationMatrix()
}

void firstPointReckoning(struct DataPoint dp)
{

}