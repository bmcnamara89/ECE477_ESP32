#include <stdio.h>
#include <math.h>
#include <time.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "data_structures.h"
#include "bluetooth.h"
#include "sensor.h"
#include "battery_lut.h"
#include "colors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//static const char *TAG = "i2c-example";
#define I2C_SLAVE_ADDR	    0x4A
#define TIMEOUT_MS		    1000
#define DELAY_MS		    1000
#define TIMER_DIVIDER       (64)
#define SWING_TIME          0.4 //s
#define TRANSMIT_TIME       2.0 //s
#define FOREHAND_THRESHOLD  40.0 //m/s/s
#define BACKHAND_THRESHOLD  30.0 //m/s/s
#define SERVE_THRESHOLD     25.0 //m/s/s
#define IMPACT_THRESH       20.0
#define GPIO_RED            25 //27
#define GPIO_GREEN          27 //26
#define GPIO_BLUE           26 //25
#define GPIO_INTR           4

//Function Declarations
//static void buttonHandler();
float qToFloat(uint16_t fixedPointValue, uint8_t qPoint);
float q8ToFloat(uint16_t fixedPointValue);
float q14ToFloat(uint16_t fixedPointValue);
void setup_GPIO();
void configure_LED();
void set_LED(uint8_t color);
void mode_LED(uint8_t mode);
void setup_button();
void button_task(void *params);
void setup_BNO_I2C();
void setup_swing_Timer();
void printDP(struct DataPoint dp);
float getMagnitude(float x, float y, float z);
void printDPS(struct DataPoint *dps, int numDPs);
void fakeReckoning(struct DataPoint *dps, int numDPs);
void print_buffer(struct DataOut* data, uint16_t len);
void deadReckoning(struct DataPoint *dps, int numDPs);
float pointReckoning(struct DataPoint dp);
void firstPointReckoning(double t);
float set_thresh(int mode);

//Global Variables
struct Coordinates globalPosition;
struct Coordinates globalVelocity;
double globalLastTime;
double globalTimeSinceLastPoint;
float rotationMatrix [4][4];
xQueueHandle interruptQueue;

static void IRAM_ATTR buttonHandler(void *args)
{
    int pinNumber = (int) args;
    xQueueSendFromISR(interruptQueue, &pinNumber, NULL);
}

float qToFloat(uint16_t fixedPointValue, uint8_t qPoint)
{
	short signedPointVal = fixedPointValue;
	float qFloat = signedPointVal * pow(2, (-1.0*qPoint));
	return qFloat;
}


float q8ToFloat(uint16_t fixedPointValue)
{
	short signedPointVal = fixedPointValue;
	float qFloat = signedPointVal * 0.00390625;
	return qFloat;
}


float q14ToFloat(uint16_t fixedPointValue)
{
	short signedPointVal = fixedPointValue;
	float qFloat = signedPointVal * 0.00006103515625;
	return qFloat;
}


void setup_GPIO()
{
    configure_LED();
    //setup_button();
}

void configure_LED()
{
    gpio_reset_pin(GPIO_RED);
    gpio_set_direction(GPIO_RED, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_GREEN);
    gpio_set_direction(GPIO_GREEN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_BLUE);
    gpio_set_direction(GPIO_BLUE, GPIO_MODE_OUTPUT);
}

void set_LED(uint8_t color)
{
    gpio_set_level(GPIO_RED, 0);
    gpio_set_level(GPIO_GREEN, 0);
    gpio_set_level(GPIO_BLUE, 0);

    if(color == WHITE || color == RED || color == YELLOW || color == MAGENTA)
    {
        gpio_set_level(GPIO_RED, 1);
    }
    if(color == WHITE || color == GREEN || color == YELLOW || color == CYAN)
    {
        gpio_set_level(GPIO_GREEN, 1);
    }
    if(color == WHITE || color == BLUE || color == MAGENTA || color == CYAN)
    {
        gpio_set_level(GPIO_BLUE, 1);
    }
}

void mode_LED(uint8_t mode)
{
    if(mode == 1)
    {
        set_LED(MAGENTA); //backhand
    }
    else if(mode == 2)
    {
        set_LED(CYAN); //forehand
    }
    else if(mode == 3)
    {
        set_LED(GREEN); //serve
    }
    else
    {
        set_LED(WHITE); //UNDEFINED MODE
    }
}

void setup_button() 
{
    gpio_pad_select_gpio(GPIO_INTR);
    gpio_set_direction(GPIO_INTR, GPIO_MODE_INPUT);
    gpio_pulldown_en(GPIO_INTR);
    gpio_pullup_dis(GPIO_INTR);
    gpio_set_intr_type(GPIO_INTR, GPIO_INTR_POSEDGE);

    interruptQueue = xQueueCreate(1, sizeof(int));
    xTaskCreate(button_task, "Button_Task", 2048, NULL, 1, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INTR, buttonHandler, (void *)GPIO_INTR);
}

void button_task(void *params)
{
    int pinNumber;
    int count = 0;
    while(1)
    {
        if(xQueueReceive(interruptQueue, &pinNumber, portMAX_DELAY))
        {
            count++;
            printf("Button Task #%d\n", count);
            set_LED(WHITE);
        }
    }
}

void setup_BNO_I2C()
{
	uint8_t set_feature_linaccel[21]    = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x04, 0x0, 0x0, 0x0, 0x20, 0x3D, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_gravity[21]     = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x06, 0x0, 0x0, 0x0, 0x20, 0x3D, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_quaternion[21]  = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x05, 0x0, 0x0, 0x0, 0x20, 0x3D, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

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
    //GPIO
    setup_GPIO();
    set_LED(RED); //Set to Red Initially
    printf("GPIO Init...\n");

    //I2C
	uint8_t rx_data[23];
    setup_BNO_I2C();
    printf("I2C Init...\n");

    //Timer
    setup_swing_Timer();
    printf("Timer Init...\n");

    //Bluetooth
    init_ble();
    printf("Bluetooth Init...\n");

    //BatteryRead
    config_battery_read_pin();
    printf("Battery Read Pin Init...\n");

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
    int swingState = 0; //0 = not in swing; 1 = in swing; 2 = transmitting swing
    double swingStartTime = 0.0;

    struct DataPoint *dps = NULL;
    int dpnum = 0;

    int mode = -1;
    float thresh = 100.0; //m/s/s (should not remain this value)
    
    while(1)
    {
        vTaskDelay(DELAY_MS/portTICK_RATE_MS);

        if(get_ble())
        {
            set_LED(BLUE);
        }
        else
        {
            set_LED(RED);
        }

        if(get_start())
        {
            mode = get_mode();
            mode_LED(mode);
            thresh = set_thresh(mode);
            
            while (get_end_of_session() == 0) 
            {
                i2c_master_read_from_device(I2C_NUM_0, I2C_SLAVE_ADDR, rx_data, 23, TIMEOUT_MS/portTICK_RATE_MS);

                //printf("Here\n");
                
                if((swingState == 0 || swingState == 1) && rx_data[9] == 0x04) //Linear Acceleration
                {
                    gotLinAccel = 1;
                    // uint8_t q = 8;
                    uint16_t x_raw = (rx_data[14] << 8) | rx_data[13];
                    uint16_t y_raw = (rx_data[16] << 8) | rx_data[15];
                    uint16_t z_raw = (rx_data[18] << 8) | rx_data[17];
                    // x = qToFloat(x_raw, q);
                    // y = qToFloat(y_raw, q);
                    // z = qToFloat(z_raw, q);
                    x = q8ToFloat(x_raw);
                    y = q8ToFloat(y_raw);
                    z = q8ToFloat(z_raw);

                    linaccel.x = x;
                    linaccel.y = y;
                    linaccel.z = z;

                    if(swingState == 0 && getMagnitude(x, y, z) > thresh) //Start of Swing Detected
                    {
                        set_LED(YELLOW);
                        swingState = 1;
                        gotGravity = 0;
                        gotQuaternions = 0;
                        swingNum++;

                        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); 
                        swingStartTime = 0.0; //timerSec;

                        gotGravity = 0;
                        gotQuaternions = 0;
        
                        dps = (struct DataPoint *) malloc(sizeof(struct DataPoint) * 50);
                    }
                }
                else if(swingState == 1 && rx_data[9] == 0x06) //Gravity
                {
                    gotGravity = 1;

                    // uint8_t q = 8;
                    uint16_t x_raw = (rx_data[14] << 8) | rx_data[13];
                    uint16_t y_raw = (rx_data[16] << 8) | rx_data[15];
                    uint16_t z_raw = (rx_data[18] << 8) | rx_data[17];
                    // x = qToFloat(x_raw, q);
                    // y = qToFloat(y_raw, q);
                    // z = qToFloat(z_raw, q);
                    x = q8ToFloat(x_raw);
                    y = q8ToFloat(y_raw);
                    z = q8ToFloat(z_raw);

                    grav.x = x;
                    grav.y = y;
                    grav.z = z;

                }
                else if(swingState == 1 && rx_data[9] == 0x05) //Quaternion
                {
                    gotQuaternions = 1;

                    // uint8_t q = 14;
                    uint16_t r_raw = (rx_data[20] << 8) | rx_data[19];
                    uint16_t i_raw = (rx_data[14] << 8) | rx_data[13];
                    uint16_t j_raw = (rx_data[16] << 8) | rx_data[15];
                    uint16_t k_raw = (rx_data[18] << 8) | rx_data[17];
                    // r = qToFloat(r_raw, q);
                    // i = qToFloat(i_raw, q);
                    // j = qToFloat(j_raw, q);
                    // k = qToFloat(k_raw, q);
                    r = q14ToFloat(r_raw);
                    i = q14ToFloat(i_raw);
                    j = q14ToFloat(j_raw);
                    k = q14ToFloat(k_raw);

                    quat.r = r;
                    quat.i = i;
                    quat.j = j;
                    quat.k = k;
                }

                if(swingState == 1 && gotQuaternions && gotLinAccel && gotGravity)
                {
                    gotLinAccel = 0;
                    gotGravity = 0;
                    gotQuaternions = 0;

                    dp.quat = quat;
                    dp.grav = grav;
                    dp.linaccel = linaccel;
                    dp.speed = getMagnitude(globalVelocity.x, globalVelocity.y, globalVelocity.z);
                    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec); 
                    dp.time = timerSec;
                    
                    //printDP(dp);

                    if(swingState == 1)
                    {
                        dps[dpnum] = dp;
                        dpnum++;


                        //printf("Swing %d: from %.3f to %.3f of Length %.3f seconds\n", swingNum, swingStartTime, timerSec, timerSec - swingStartTime);
                    }
                }
                
                timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &timerSec);
                if(swingState == 1 && timerSec - swingStartTime > SWING_TIME) //END Swing 
                {
                    swingState = 2;                    

                    //fakeReckoning(dps, dpnum);
                    deadReckoning(dps, dpnum); //store in outputdatapoints

                    free(dps);
                    dpnum = 0; 
                }
                else if(swingState == 2)
                {
                    if(timerSec > SWING_TIME + TRANSMIT_TIME) //ready to accept next swing
                    {
                        swingState = 0;
                        mode_LED(get_mode());
                    }
                    else
                    {
                        vTaskDelay(DELAY_MS/portTICK_RATE_MS);
                    }
                }
                else if(swingState == 0 && timerSec == 5.0)
                {
                    printf("ERROR REACHED");
                    set_LED(WHITE);
                }
            }
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
    struct DataOut * outputData;  
    outputData = (struct DataOut *) malloc(sizeof(struct DataOut) * numDPs);
    float maxDiff = 0;
    float currDiff = 0;
    float impactTime = 0.0;
    float contactSpeed = 0.0;

    for(int i = 0; i < numDPs; i++)
    {
        outputData[i].pos.x = dps[i].linaccel.x;
        outputData[i].pos.y = dps[i].linaccel.y;
        outputData[i].pos.z = dps[i].linaccel.z;
        outputData[i].quat.r = dps[i].quat.r;
        outputData[i].quat.i = dps[i].quat.i;
        outputData[i].quat.j = dps[i].quat.j;
        outputData[i].quat.k = dps[i].quat.k;
        outputData[i].time = dps[i].time;

        if(i > 0)
        {   
            currDiff = abs(getMagnitude(dps[i].linaccel.x, dps[i].linaccel.y, 0) - getMagnitude(dps[i-1].linaccel.x, dps[i-1].linaccel.y, 0));
            if(currDiff > maxDiff)
            {
                maxDiff = currDiff;
                impactTime = dps[i].time;
                contactSpeed = dps[i].speed;
            }
        }
    }

    if(maxDiff < IMPACT_THRESH)
    {
        impactTime = -1.0;
    }

    //printf("Buffer Filled: %d Data Points\n", numDPs);
    //print_buffer(outputData, numDPs);

    set_transmit_buffer(outputData, numDPs, impactTime, contactSpeed);

    free(outputData);
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
    outputData = (struct DataOut *) malloc(sizeof(struct DataOut) * numDPs); 
    float maxDiff = 0;
    float currDiff = 0;
    float impactTime = 0.0;
    float impactSpeed = 0.0;
    float velmag = 0.0;

    for(int i = 0; i < numDPs; i++)
    {
        //Call Dead Reckoning on Each Point
        if(i > 0)
        {
            velmag = pointReckoning(dps[i]);
        }
        else
        {
            firstPointReckoning(dps[i].time); //First Data Point
        }
        outputData[i].pos.x = globalPosition.x;
        outputData[i].pos.y = globalPosition.y;
        outputData[i].pos.z = globalPosition.z;
        outputData[i].quat.r = dps[i].quat.r;
        outputData[i].quat.i = dps[i].quat.i;
        outputData[i].quat.j = dps[i].quat.j;
        outputData[i].quat.k = dps[i].quat.k;
        outputData[i].time = dps[i].time;

        //PrintCurrentPosition(globalPosition); //Used for Graphing on Processing
            if(i > 0)
        {   
            currDiff = abs(getMagnitude(dps[i].linaccel.x, dps[i].linaccel.y, 0) - getMagnitude(dps[i-1].linaccel.x, dps[i-1].linaccel.y, 0));
            if(currDiff > maxDiff)
            {
                maxDiff = currDiff;
                impactTime = dps[i].time;
                impactSpeed = velmag;
            }
        }
    }

    if(maxDiff < IMPACT_THRESH)
    {
        impactTime = -1.0;
        impactSpeed = -1.0;
    }

    //printf("Buffer Filled: %d Data Points\n", numDPs);
    //print_buffer(outputData, numDPs);
    set_transmit_buffer(outputData, numDPs, impactTime, impactSpeed);

    free(outputData);
}

float pointReckoning(struct DataPoint dp)
{
    ConvertQuaternionToRotationMatrix(dp.quat);
    struct Coordinates correctedAccel = ConvertLocalToGlobalCoords(dp.linaccel);
    globalTimeSinceLastPoint = dp.time - globalLastTime;
    globalLastTime = dp.time;
    float velmag = UpdatePosition(correctedAccel);
    return velmag;
}

//Initialize Global Swing Values on first Data Point
void firstPointReckoning(double t)
{
    globalVelocity.x = 0.0;
    globalVelocity.y = 0.0;
    globalVelocity.z = 0.0;

    globalPosition.x = 0.0;
    globalPosition.y = 0.0;
    globalPosition.z = 0.0;

    globalLastTime = t;
    globalTimeSinceLastPoint = t;
}

float set_thresh(int mode)
{
    float thresh;
    if(mode == 1)
    {
        thresh = BACKHAND_THRESHOLD; //backhand
    }
    else if(mode == 2)
    {
        thresh = FOREHAND_THRESHOLD; //forehand
    }
    else if(mode == 3)
    {
        thresh = SERVE_THRESHOLD; //serve
    }
    else
    {
        thresh = -1.0; //UNDEFINED MODE
    }

    return thresh;
}