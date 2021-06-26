/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/i2c.h"

#include "esp_log.h"
#define TAG "main"


#define WM8960_I2C_DEV_ADDR 0x34


esp_err_t wm8960_write_reg(uint8_t reg, uint16_t dat)
{
	uint8_t b[2];
	int ret;

	b[0] = (reg<<1)|((uint8_t)((dat>>8)&0x0001));
	b[1] = (uint8_t)(dat&0x00FF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, WM8960_I2C_DEV_ADDR  | I2C_MASTER_WRITE, 1);
    i2c_master_write(cmd, b, 2, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

	if(ret != ESP_OK)
	{
		ESP_LOGE(TAG, "wm8960 write reg failed");
	}
	return ret;
}


static inline void i2c_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 0;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

	i2c_driver_install(I2C_NUM_0, conf.mode);
	i2c_param_config(I2C_NUM_0, &conf);
	
    vTaskDelay(100 / portTICK_RATE_MS);
}


void app_main()
{
	i2c_init();
	
	ESP_LOGI(TAG, "esp8266 started !");
	
	// Reset Device
	wm8960_write_reg(0x0f, 0x0000);
	//Set Power Source
	wm8960_write_reg(0x19, 1<<8 | 1<<7 | 1<<6);
	wm8960_write_reg(0x1A, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3);
	wm8960_write_reg(0x2F, 1<<3 | 1<<2);

	//Configure clock
	//MCLK->div1->SYSCLK->DAC/ADC sample Freq = 25MHz(MCLK)/2*256 = 48.8kHz
	wm8960_write_reg(0x04, 0x0000);

	//Configure ADC/DAC
	wm8960_write_reg(0x05, 0x0000);

	//Configure audio interface
	//I2S format 16 bits word length
	wm8960_write_reg(0x07, 0x0002);

	//Configure HP_L and HP_R OUTPUTS
	wm8960_write_reg(0x02, 0x006F | 0x0100);  //LOUT1 Volume Set
	wm8960_write_reg(0x03, 0x006F | 0x0100);  //ROUT1 Volume Set

	//Configure SPK_RP and SPK_RN
	wm8960_write_reg(0x28, 0x007F | 0x0100); //Left Speaker Volume
	wm8960_write_reg(0x29, 0x007F | 0x0100); //Right Speaker Volume

	//Enable the OUTPUTS
	wm8960_write_reg(0x31, 0x00F7); //Enable Class D Speaker Outputs

	//Configure DAC volume
	wm8960_write_reg(0x0a, 0x00FF | 0x0100);
	wm8960_write_reg(0x0b, 0x00FF | 0x0100);

    while(1)
	{
		ESP_LOGI(TAG, "Hello there");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
