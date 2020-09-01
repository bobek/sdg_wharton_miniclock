/*
 * brightnessControl.c
 *
 *  Created on: 29 Mar 2020
 *      Author: Steve
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "brightnessControl.h"
#include "esp_log.h"

static const char *TAG = "Brightness Control";

// ADC Setting
#define DEFAULT_VREF    1100
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (4)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_FADE_TIME    1000
#define LEDC_HS_CH1_GPIO       (33)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_channel_t channel = ADC_CHANNEL_0;

static void check_efuse(void);
static void print_char_val_type(esp_adc_cal_value_t val_type);

static esp_adc_cal_characteristics_t *adc_chars;

ledc_channel_config_t ledc_channel_face = {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
};

ledc_channel_config_t ledc_channel_matrix = {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
};
/* Configure Timers for the LED Controller (Allows PWM Control of LEDS) */
ledc_timer_config_t ledc_timer = {
   .duty_resolution = LEDC_TIMER_9_BIT, // resolution of PWM duty
   .freq_hz = 100000,                    // frequency of PWM signal
   .speed_mode = LEDC_HS_MODE,           // timer mode
   .timer_num = LEDC_HS_TIMER,            // timer index
   .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
};

//----------------------------------------------------------
// Configure ADC
//----------------------------------------------------------
void adc_setup(void)
{
check_efuse();

	// Now configure ADC 1
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(channel, atten);

	//Characterize ADC
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);
}

//-------------------------------------------------------------------------
//  NAME
//      brightness_task
//
//  DESCRIPTION
//!     Do ADC and update the brightness of the LED Display
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void brightness_task( void * pvParameters )
{
	uint32_t adc_reading = 0;
	uint32_t scaled_adc_reading = 0;
	for( ;; )
	{
		adc_reading = adc1_get_raw((adc1_channel_t)channel);
	 	//Convert adc_reading to voltage in mV
	 	//uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	 	//ESP_LOGI(TAG, "Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
	 	//ESP_LOGI(TAG, "ADC%d CH%d Raw: %d\t\n", unit, channel, adc_reading);
	 	// ADC Reading from 0-4096
	 	// Should convert to 511-0
		/*
	 	if(adc_reading < 100)
	 	{
	 		scaled_adc_reading = adc_reading;
	 	}
	 	else if(adc_reading < 1000)
	 	{
	 		scaled_adc_reading = ((adc_reading-100) / 2) + 100; // Max of 550
	 	}
	 	else // if less than 4096
	 	{
	 		scaled_adc_reading =  ((adc_reading-1000) / 8) + 550; // Max of 1010
	 	}
	 	if(scaled_adc_reading > 1010)
	 	{
	 		scaled_adc_reading = 1010;
	 	}
	 	// Now 0-1023
	 	scaled_adc_reading = 1015 - scaled_adc_reading;*/

		if(adc_reading < 10)
		{
			scaled_adc_reading =  511;
		}
		else
		{
			if(adc_reading > 4000)
			{
				adc_reading = 4000;
			}
			adc_reading = adc_reading - 10;
			adc_reading = adc_reading / 12;
			scaled_adc_reading = 511-adc_reading;

		}

	 	ESP_LOGI(TAG, "ADC: %d New duty %d\t\n", adc_reading,scaled_adc_reading);

	 	ledc_set_fade_with_time(ledc_channel_face.speed_mode, ledc_channel_face.channel, scaled_adc_reading, LEDC_TEST_FADE_TIME);
	 	ledc_fade_start(ledc_channel_face.speed_mode, ledc_channel_face.channel, LEDC_FADE_NO_WAIT);

	 	ledc_set_fade_with_time(ledc_channel_matrix.speed_mode, ledc_channel_matrix.channel, scaled_adc_reading, LEDC_TEST_FADE_TIME);
	 	ledc_fade_start(ledc_channel_matrix.speed_mode, ledc_channel_matrix.channel, LEDC_FADE_NO_WAIT);
	 	vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

//-------------------------------------------------------------------------
//  NAME
//      initialisebrightnessControlPWM
//
//  DESCRIPTION
//!     Set up ledc for PWM operation
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void initialisebrightnessControlPWM(void)
{
// Initialize fade service.
    ledc_fade_func_install(0);


 // Set configuration of timer0 for high speed channels
 	 ESP_ERROR_CHECK( ledc_timer_config(&ledc_timer));
 	 ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel_face));
 	 ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel_matrix));

 	 ESP_ERROR_CHECK( ledc_set_duty(ledc_channel_face.speed_mode, ledc_channel_face.channel,  511));
 	 ESP_ERROR_CHECK( ledc_update_duty(ledc_channel_face.speed_mode, ledc_channel_face.channel));
 	 ESP_ERROR_CHECK( ledc_set_duty(ledc_channel_matrix.speed_mode, ledc_channel_matrix.channel,  511));
 	 ESP_ERROR_CHECK( ledc_update_duty(ledc_channel_matrix.speed_mode, ledc_channel_matrix.channel));
}

//-------------------------------------------------------------------------
//  NAME
//      pausebrightnessControlPWM
//
//  DESCRIPTION
//!     Reset timers
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void pausebrightnessControlPWM(void)
{
	ledc_timer_pause(ledc_channel_matrix.speed_mode, ledc_channel_face.channel);
}

//-------------------------------------------------------------------------
//  NAME
//      resumebrightnessControlPWM
//
//  DESCRIPTION
//!     Reset timers
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void resumebrightnessControlPWM(void)
{
	ledc_timer_resume(ledc_channel_matrix.speed_mode, ledc_channel_face.channel);
}

//-------------------------------------------------------------------------
//  NAME
//      check_efuse
//
//  DESCRIPTION
//!     Checks if the ESP32 had characterisation values burned in
//!	    This is for Two Point or Vref Values
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    	ESP_LOGI(TAG, "eFuse Two Point: Supported\n");
    } else {
    	ESP_LOGI(TAG, "eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    	ESP_LOGI(TAG, "eFuse Vref: Supported\n");
    } else {
    	ESP_LOGI(TAG, "eFuse Vref: NOT supported\n");
    }
}
//-------------------------------------------------------------------------
//  NAME
//      print_char_val_type
//
//  DESCRIPTION
//!     Prints how the ADC will be characterised
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    	ESP_LOGI(TAG, "Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    	ESP_LOGI(TAG, "Characterized using eFuse Vref\n");
    } else {
    	ESP_LOGI(TAG, "Characterized using Default Vref\n");
    }
}
