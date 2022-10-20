#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>

//static const char *TAG = "i2c-example";
#define I2C_SLAVE_ADDR	0x4A
#define TIMEOUT_MS		1000
#define DELAY_MS		1000


float qToFloat(uint16_t fixedPointValue, uint8_t qPoint)
{
	short signedPointVal = fixedPointValue;
	float qFloat = signedPointVal * pow(2, (-1.0*qPoint));
	//qFloat *= 2 ^ (qPoint*-1);
	return qFloat;
}

void app_main() {
	uint8_t rx_data[23];
	uint8_t set_feature_linaccel[21]    = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x04, 0x0, 0x0, 0x0, 0x60, 0xEA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_gravity[21]     = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x06, 0x0, 0x0, 0x0, 0x60, 0xEA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t set_feature_quaternion[21]  = {0x15, 0x00, 0x02, 0x00, 0xFD, 0x05, 0x0, 0x0, 0x0, 0x60, 0xEA, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

	//uint8_t get_feature_request[6] = {0x6, 0x00, 0x02, 0x01, 0xFE, 0x01};
	//uint8_t get_id[6] = {0x06, 0x00, 0x02, 0x00, 0xF9, 0x00};

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

	//feature request
	//i2c_master_write_to_device(I2C_NUM_0, I2C_SLAVE_ADDR, get_feature_request, 6, TIMEOUT_MS/portTICK_RATE_MS);

	vTaskDelay(DELAY_MS/portTICK_RATE_MS);

	int choice = 2; //0 = none, 1=linaccel, 2=gravity, 3=quaternions

	while (1) {
		if(choice == 3)
		{
			i2c_master_read_from_device(I2C_NUM_0, I2C_SLAVE_ADDR, rx_data, 23, TIMEOUT_MS/portTICK_RATE_MS);
			if(rx_data[9] == 0x05 && rx_data[0] == 0x17)
			{
				uint8_t q = 14;
				uint16_t r_raw = (rx_data[20] << 8) | rx_data[19];
				uint16_t i_raw = (rx_data[14] << 8) | rx_data[13];
				uint16_t j_raw = (rx_data[16] << 8) | rx_data[15];
				uint16_t k_raw = (rx_data[18] << 8) | rx_data[17];
				float r = qToFloat(r_raw, q);
				float i = qToFloat(i_raw, q);
				float j = qToFloat(j_raw, q);
				float k = qToFloat(k_raw, q);

				printf("%.3f/%.3f/%.3f/%.3f\n", r, i, j, k);
			}
		}
		else
		{
			i2c_master_read_from_device(I2C_NUM_0, I2C_SLAVE_ADDR, rx_data, 19, TIMEOUT_MS/portTICK_RATE_MS);
			uint8_t q = 8;
			uint16_t x_raw = (rx_data[14] << 8) | rx_data[13];
			uint16_t y_raw = (rx_data[16] << 8) | rx_data[15];
			uint16_t z_raw = (rx_data[18] << 8) | rx_data[17];
			float x = qToFloat(x_raw, q);
			float y = qToFloat(y_raw, q);
			float z = qToFloat(z_raw, q);

			if(choice == 1 && rx_data[0] == 0x13 && rx_data[9] == 0x04 && rx_data[4] == 0xfb) //checking report types
			{
				//ESP_LOG_BUFFER_HEX(TAG, accel_data, 6);
				float mag = pow(pow(x,2)+pow(y,2)+pow(z,2),0.5);
				if(mag > 10)
				{
					printf("Start of Swing Detected\n");
				}
				//printf("Linear Acceleration: (%.3f, %.3f, %.3f)\n", x, y, z);
			}
			else if(choice == 2 && rx_data[0] == 0x13 && rx_data[9] == 0x06 && rx_data[4] == 0xfb)
			{
				//ESP_LOG_BUFFER_HEX(TAG, accel_data, 6);
				printf("Gravity Vector: (%.3f, %.3f, %.3f)\n", x, y, z);
			}
		}
		//vTaskDelay(DELAY_MS/portTICK_RATE_MS);
	}

}
