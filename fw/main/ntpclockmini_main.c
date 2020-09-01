//----------------------------------------------------------------------------------
//	Filename:         ntpclockmini_main.c
//  File Description: Mini Wharton Clock Firmware
//  Processor:        ESP32-WROOM-32
//
//  Name:             Steven Gardner
//  Company:          SDG Electronics
//  Creation Date:    1st April 2020
//
//  Compiler:         Created in Eclipse 4.15.0 using esp-2019r2-8.2.0
//
//  Notes:
//
//----------------------------------------------------------------------------------
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "driver/timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "brightnessControl.h"
#include <cJSON.h>

static const char *TAG = "Mini Wharton Clock";

// Helpful time related macros
#define SECONDS(X) ((X) * (1000 / portTICK_RATE_MS))
#define MINUTES(Y) SECONDS(60 * (Y) )

RTC_DATA_ATTR static int boot_count = 0;

#define DATA_CLK			21
#define DATA_BITS   		22
#define OUTPUT_ENABLE_INV	4
#define LATCH_CATHODES		2
#define LATCH_ANODES		15
#define MRESET_INV			32

#define TESTPIN				23

// SPI Pin Config
#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 22
#define PIN_NUM_CLK  21
#define PIN_NUM_CS   -1



/** PIN CONFIGURATION
 * Firstly the shift registers
 * IO23 = DATA CLOCK
 * IO22 = DATA BITSTREAM
 * IO4  = _OUTPUT ENABLE
 * IO2  = LATCH FACE (CATHODES)
 * IO15 = LATCH ROW (ANODES)
 * IO32 = _MRESET
 *
 * Then the buttons
 * IO34 = UP
 * IO35 = DOWN
 * IO18 = RIGHT
 * IO5  = LEFT
 * IO17 = OK
 * IO16 = BACK
 *
 * Analogue Inputs
 * SENSOR_VP = Ambient Light Sensor
**/
#define OPT_TENS 2
#define OPT_ONES 1
#define OPT_BOTH 0

#define REFRESH_COMPLETE 0
#define REFRESH_ANODES   1
#define REFRESH_CATHODES 2
#define REFRESH_DISABLE_ANODES 3

const uint8_t segmentMap[] = {
// XGFEDCBA
 0b00111111, // 0
 0b00000110, // 1
 0b01011011, // 2
 0b01001111, // 3
 0b01100110, // 4
 0b01101101, // 5
 0b01111101, // 6
 0b00000111, // 7
 0b01111111, // 8
 0b01101111, // 9
 0b01110111, // A
 0b01111100, // B
 0b00111001, // C
 0b01011110, // D
 0b01111001, // E
 0b01110001  // F
};

const char dowNames[7][10] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char monthNames[12][10] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const char dateEnds[31][3] = {"st", "nd", "rd", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th", "th", "st"};


// Wifi settings will be taken from "menucofing"
#define WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

static int s_wifi_retry_num = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/**             23 22 21 20 19 18 17 16   15 14 13 12 11 10   9  8  7  6  5  4  3  2    1  0
               [0  1  2  3  4  5  6  7    8  9  10 11 12 13   14 15 16 17 18 19 20 21   22 23]
        S      |-     SEGMENTS      -|   |-  MARKERS   -|    |-    SECONDS        -|   NC NC
HOURS,T 0  1    A  B  C  D  E  F  G       1  2  3  4  5  6    0  1  2  3  4  5  6  7
HOURS,O 1  2    A  B  C  D  E  F  G       7  8  9  10 11 12   8  9  10 11 12 13 14 15
MINS,T  2  3    A  B  C  D  E  F  G                           16 17 18 19 20 21 22 23
MINS,O  3  4    A  B  C  D  E  F  G                           24 25 26 27 28 29 30 31
SECS,T  4  5    A  B  C  D  E  F  G                           32 33 34 35 36 37 38 39
SECS,O  5  6    A  B  C  D  E  F  G                           40 41 42 43 44 45 46 47
COLON   6  7                        COL                       48 49 50 51 52 53 54 55
        7  8                                                  56 57 58 59


**/

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
time_t now = 0;
struct tm timeinfo = { 0 };
uint32_t seconds_bitmap = 0;  // used to temporarily store the seconds bitmap
uint32_t bitstream_face = 0;  // this is the data to be shifted out
uint16_t multiplex_state = 0; // state 0 to 7 for 1/8 multiplexing
uint8_t  time_to_multiplex_anodes = 0;
uint8_t  time_to_multiplex_cathodes = 0;
uint8_t  time_update = 0;
uint8_t	 multiplex_anodeORcathode = REFRESH_DISABLE_ANODES;
uint32_t twentyfourbits = 0;
uint32_t eightbits = 0;
uint32_t eightzeroes = 0;
spi_transaction_t t;

uint8_t g_last = 0;



uint16_t task_core = 0;
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void create_timer_task_core1(void *param);
void obtain_time(void);
void initialise_sntp(void);
void initialise_wifi(void);
void wifi_init_sta(void);
void app_init(void);
void init_timer(int timer_period_us);
void timer_isr(void* arg);
void multiplex_timer_callback(void* arg);
void onesec_timer_callback(void* arg);

uint16_t display_multiplex_prepare_8bits_anodes(uint16_t multiplex_no);
uint32_t display_multiplex_prepare_24bits(uint16_t multiplex_no);
uint32_t calculate_seconds_bitmap(uint16_t multiplex_stage, uint16_t seconds);
uint16_t decimalToBCD(uint16_t decimal, uint16_t ones_tens);

void spi_pre_transfer_callback(spi_transaction_t *t);
void spi_post_transfer_callback(spi_transaction_t *t);

//-----------------------------------------------------------------------------
// Structs
//-----------------------------------------------------------------------------
time_t now;
struct tm timeinfo;



spi_device_handle_t spi;
spi_bus_config_t buscfg={
    .miso_io_num=PIN_NUM_MISO,
    .mosi_io_num=PIN_NUM_MOSI,
    .sclk_io_num=PIN_NUM_CLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=24
};

spi_device_interface_config_t devcfg={
	.command_bits		= 0,
	.dummy_bits			= 0,
	.mode				= 0,                //SPI mode 0
	.duty_cycle_pos		= 0,
	.cs_ena_posttrans 	= 0,
	.cs_ena_pretrans	= 0,
	.clock_speed_hz		= 5000000,               //Clock out at 5 MHz
	.spics_io_num		= PIN_NUM_CS,       //CS pin
	.flags				= 0,
	.queue_size			= 1,                    //We want to be able to queue 1 transactions at a time
	.pre_cb             = spi_pre_transfer_callback,
	.post_cb			= spi_post_transfer_callback,
};

static intr_handle_t s_timer_handle;
//-----------------------------------------------------------------------------
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

//-------------------------------------------------------------------------
//  NAME
//      create_timer_task_core1
//
//  DESCRIPTION
//!     Display Refresh Timer Task, running on Core 1
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void create_timer_task_core1(void *param)
{
    while(1)
     {
        init_timer(500);		// Try setting timer to 1000=1000Hz
        ESP_LOGI(TAG, "Creating Timer ISR:%d\r\n",xPortGetCoreID());
        vTaskDelete( NULL );
    }
}

//-------------------------------------------------------------------------
//  NAME
//      app_main
//
//  DESCRIPTION
//!     Main Loop
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void app_main()
{
    ++boot_count;
    ESP_LOGI(TAG, "app_main() is starting...");
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

	app_init();

	ESP_LOGI(TAG, "Initialisation Complete");

	spi_bus_initialize(HSPI_HOST, &buscfg, 0);			// Initialize the SPI bus
	spi_bus_add_device(HSPI_HOST, &devcfg, &spi);		// Attach the device to the SPI bus

    memset(&t, 0, sizeof(t));       //Zero out the transaction

    ESP_LOGI(TAG, "Checking if Time and Date is set");

    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        time(&now);
    }

    char strftime_buf[64];

    ESP_LOGI(TAG, "Setting Time Zone");

    // Set timezone to GMT and print local time
    setenv("TZ", "GMT+0BST-1,M3.5.0/01:00:00,M10.5.0/02:00:00", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

    ESP_LOGI(TAG, "Creating Timer Task for Multiplexing");
    xTaskCreatePinnedToCore(create_timer_task_core1, "TimerTask", 8192, NULL, 2, NULL, 1);
    ESP_LOGI(TAG, "Creating Brightness Adjustment Task");
    xTaskCreate(brightness_task, "BrightnessTask", 8192, NULL, 20, NULL);

    ESP_LOGI(TAG, "Starting Main Task...");

	while (1)
	{
		if(time_update == 1)
		{
			time_update = 0;
			time(&now);
			localtime_r(&now, &timeinfo);
		}
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}


}

void obtain_time(void)
{
    // wait for time to be set

    int retry = 0;
    const int retry_count = 30;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count+1) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
    // Note - time will be checked over WiFi every 10 minutes (600000ms) as set in SDK Config -> LWIP -> SNTP
}

void initialise_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "10.0.0.250");     // Use "pool.ntp.org" if no local NTP server 
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
    sntp_init();
}

void initialise_wifi(void)
{
    // initialize NVS flash, needed for WI-FI usage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}

//-------------------------------------------------------------------------
//  NAME
//      app_init
//
//  DESCRIPTION
//!     Initialises module
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void app_init(void)
{
	//----------------------------------------------------------
	// Configure GPIO
	//----------------------------------------------------------
	gpio_set_direction(LATCH_CATHODES, GPIO_MODE_OUTPUT);
	gpio_set_level(LATCH_CATHODES, 0);
	gpio_pad_select_gpio(LATCH_ANODES);
	gpio_set_direction(LATCH_ANODES, GPIO_MODE_OUTPUT);
	gpio_set_level(LATCH_ANODES, 0);
	gpio_set_direction(MRESET_INV, GPIO_MODE_OUTPUT);
	gpio_set_level(MRESET_INV, 0);
	gpio_set_level(LATCH_CATHODES, 1);
	gpio_set_level(LATCH_ANODES, 1);
	gpio_set_level(MRESET_INV, 1);

	adc_setup();


	//----------------------------------------------------------
	// Configure Timers
	//----------------------------------------------------------
    const esp_timer_create_args_t multiplex_timer_args = {
            .callback = &multiplex_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "multiplex"
    };
    esp_timer_handle_t multiplex_timer;
    ESP_ERROR_CHECK(esp_timer_create(&multiplex_timer_args, &multiplex_timer));
    /* The timer has been created but is not running yet */

    const esp_timer_create_args_t onesec_timer_args = {
            .callback = &onesec_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "onesec"
    };
    esp_timer_handle_t onesec_timer;
    ESP_ERROR_CHECK(esp_timer_create(&onesec_timer_args, &onesec_timer));

    /* Start the timers */
    // ESP_ERROR_CHECK(esp_timer_start_periodic(multiplex_timer, 1500));	//0.00125s - 800Hz 1250
    ESP_ERROR_CHECK(esp_timer_start_periodic(onesec_timer, 250000));	//1s - 4Hz

	//----------------------------------------------------------
	// Configure PWM
	//----------------------------------------------------------
    initialisebrightnessControlPWM();

 	//----------------------------------------------------------
 	// Configure WiFi and SNTP
 	//----------------------------------------------------------
    initialise_wifi();
    initialise_sntp();
}

//-------------------------------------------------------------------------
//  NAME
//      init_timer
//
//  DESCRIPTION
//!     Initialises Timer 0 from Timer Group 0 with interrupt
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void init_timer(int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 80   /* 1 us per tick */
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

//-------------------------------------------------------------------------
//  NAME
//      timer_isr
//
//  DESCRIPTION
//!     The Timer 0 interrupt
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void timer_isr(void* arg)
{

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    task_core = xPortGetCoreID();

		twentyfourbits = display_multiplex_prepare_24bits(multiplex_state);
		eightbits = display_multiplex_prepare_8bits_anodes(multiplex_state);

		multiplex_state++;
		if(multiplex_state > 7) multiplex_state = 0;

		gpio_set_level(LATCH_CATHODES, 0);
		gpio_set_level(LATCH_ANODES, 0);
		gpio_set_level(MRESET_INV, 0);
		gpio_set_level(LATCH_CATHODES, 1);
		gpio_set_level(LATCH_ANODES, 1);
		gpio_set_level(MRESET_INV, 1);
		gpio_set_level(LATCH_CATHODES, 0);
		gpio_set_level(LATCH_ANODES, 0);


			if(multiplex_state < 8)
			{
				multiplex_anodeORcathode = REFRESH_CATHODES;
				memset(&t, 0, sizeof(t));       //Zero out the transaction
		    	t.length=24;                     // Command is 24 bits
		    	t.tx_buffer=&twentyfourbits;     // Data in the buffer
		    	spi_device_queue_trans(spi, &t, portMAX_DELAY);
			}

			while(multiplex_anodeORcathode == REFRESH_CATHODES)
			{

			};

			if(multiplex_state < 8)
			{
				multiplex_anodeORcathode = REFRESH_ANODES;
				memset(&t, 0, sizeof(t));       //Zero out the transaction
		    	t.length=8;                     // Command is 8 bits
		    	t.tx_buffer=&eightbits;         // Data in the buffer
		    	spi_device_queue_trans(spi, &t, portMAX_DELAY);
			}

			//resumebrightnessControlPWM();
}

//-------------------------------------------------------------------------
//  NAME
//      multiplex_timer_callback
//
//  DESCRIPTION
//!     Callback from high resolution timer for Multiplexing the display
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void multiplex_timer_callback(void* arg)
{
//	time_to_multiplex_cathodes = 1;
//	display_multiplex(multiplex_state++);
//	if(multiplex_state > 10) multiplex_state = 0;
}

//-------------------------------------------------------------------------
//  NAME
//      onesec_timer_callback
//
//  DESCRIPTION
//!     Callback from high resolution timer for updating the time
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void onesec_timer_callback(void* arg)
{
	time_update = 1;
}

//-------------------------------------------------------------------------
//  NAME
//      display_multiplex_prepare_8bits_anodes
//
//  DESCRIPTION
//!     Prepares 8 bits of data to the 74HC595 Shift Registers for
//      clock face multiplex rows (1-8)
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint16_t display_multiplex_prepare_8bits_anodes(uint16_t multiplex_no)
{
	uint16_t bitfield_data = 0;

	gpio_set_level(LATCH_ANODES, 0);
    // Data is latched on rising edge of clock
	if(multiplex_no > 7)
	{
		bitfield_data = 0;
	}
	else
	{
		//bitfield_data = 0x0080 >> multiplex_no;   // shifts is a value 0-7 // Normal way for shifting out manually

		bitfield_data = 0x0001 << multiplex_no;   // shifts is a value 0-7	 // Reversed for SPI
	}
    return(bitfield_data);
}

//-------------------------------------------------------------------------
//  NAME
//      display_multiplex_prepare_24bits_reversed
//
//  DESCRIPTION
//!     Call this with valid data in the tm structure to prepare 24 bits for multiplexing
//!     multiplex_no should be 0-7 for the 1 in 8 multiplexing
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint32_t display_multiplex_prepare_24bits_reversed(uint16_t multiplex_no)
{
	uint32_t temp_bitmap = 0;     // used to temporarily store the 7 seg bitmap
    bitstream_face = 0;

	// First, all markers will be lit (bit position 10-15)
	bitstream_face = bitstream_face | 0b1111110000000000;
	// Next illuminate seconds
	seconds_bitmap = calculate_seconds_bitmap(multiplex_no, timeinfo.tm_sec);
	seconds_bitmap = seconds_bitmap << 2;
	seconds_bitmap = seconds_bitmap & 0b1111111100;
	bitstream_face = bitstream_face | seconds_bitmap;

	// Now work out which digits to display
	switch (multiplex_no)
	{
		case 0: // Hours, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_hour, OPT_TENS) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 1: // Hours, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_hour, OPT_ONES) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 2: // Minutes, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_min, OPT_TENS) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 3: // Minutes, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_min, OPT_ONES) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 4: // Seconds, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_sec, OPT_TENS) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 5: // Seconds, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_sec, OPT_ONES) & 0x0F)];
			temp_bitmap = temp_bitmap << 17;  // Shift into position to copy into bitstream
			break;
		}
		case 6: // Colons
		{
			temp_bitmap = 0b10000000000000000;
			break;
		}
		default:
		{
			temp_bitmap = 0;
			break;
		}
	}

	bitstream_face = bitstream_face | (temp_bitmap & 0b111111110000000000000000);
	return bitstream_face;
}

//-------------------------------------------------------------------------
//  NAME
//      display_multiplex_prepare_24bits
//
//  DESCRIPTION
//!     Call this with valid data in the tm structure to prepare 24 bits for multiplexing
//!     multiplex_no should be 0-7 for the 1 in 8 multiplexing
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint32_t display_multiplex_prepare_24bits(uint16_t multiplex_no)
{
	uint32_t temp_bitmap = 0;     // used to temporarily store the 7 seg bitmap
    bitstream_face = 0;

	// Now work out which digits to display
	switch (multiplex_no)
	{
		case 0: // Hours, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_hour, OPT_TENS) & 0x0F)];
			break;
		}
		case 1: // Hours, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_hour, OPT_ONES) & 0x0F)];
			break;
		}
		case 2: // Minutes, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_min, OPT_TENS) & 0x0F)];
			break;
		}
		case 3: // Minutes, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_min, OPT_ONES) & 0x0F)];
			break;
		}
		case 4: // Seconds, Tens
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_sec, OPT_TENS) & 0x0F)];
			break;
		}
		case 5: // Seconds, Ones
		{
			temp_bitmap = (uint32_t)segmentMap[(decimalToBCD(timeinfo.tm_sec, OPT_ONES) & 0x0F)];
			break;
		}
		case 6: // Colons
		{
			temp_bitmap = 0b10000000;
			break;
		}
		default:
		{
			temp_bitmap = 0;
			break;
		}
	}
	temp_bitmap = temp_bitmap<<16;
	// 7 segments in <23:16>
	// Now illuminate clock face markers (bit position 13:8)
	bitstream_face = temp_bitmap | 0b11111100000000;
	// Next illuminate seconds
	seconds_bitmap = calculate_seconds_bitmap(multiplex_no, timeinfo.tm_sec);
	// Shift 2 LSBs into bit position 15:14
	temp_bitmap    = seconds_bitmap & 0b00000011;
	temp_bitmap	   = temp_bitmap << 14;
	// Then shift bits into 21:16
	seconds_bitmap = seconds_bitmap >> 2;
	seconds_bitmap = seconds_bitmap & 0b00111111;
	seconds_bitmap = seconds_bitmap | temp_bitmap;

	bitstream_face = bitstream_face | seconds_bitmap;
	return bitstream_face;
}

//-------------------------------------------------------------------------
//  NAME
//      calculate_seconds_bitmap
//
//  DESCRIPTION
//!     Calculates which LEDs to illuminate around the clock for seconds
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint32_t calculate_seconds_bitmap(uint16_t multiplex_stage, uint16_t seconds)
{
// uint16_t multiplex_stage <- value from 0 - 7
// uint16_t seconds         <- value from 0 - 59
// returns bitfield 8 bits wide of current bitmap
uint32_t bitfield_data = 0;
uint16_t seconds_segment = 0;
uint16_t seconds_modulo = 0;
    seconds_segment = (seconds+1)/8; // Value 0 to 7
	seconds_modulo = (seconds+1)%8;  // Value 0 to 7

	if(multiplex_stage > 7)
	{
		return 0;
	}

	if(multiplex_stage < seconds_segment)  			// Quick determination of all 8 LEDs lit
	{
		bitfield_data = 0xFF;
	}
	else if(multiplex_stage > seconds_segment)  	// Quick determinaton of no LEDs lit
	{
		bitfield_data = 0x00;
	}
	else
	{
		bitfield_data = 0xFF << seconds_modulo;   // shifts is a value 0-8
		bitfield_data = bitfield_data >> 0x08;
	}

	if(multiplex_stage == 0)
	{
	    bitfield_data = bitfield_data | 0b00000001; // Always illuminate "0" LED
	}

	bitfield_data = bitfield_data & 0xFF; // mask off other bits
	return bitfield_data;

}

//-------------------------------------------------------------------------
//  NAME
//      calculate_seconds_bitmap_reversed
//
//  DESCRIPTION
//!     Calculates which LEDs to illuminate around the clock for seconds
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint32_t calculate_seconds_bitmap_reversed(uint16_t multiplex_stage, uint16_t seconds)
{
// uint16_t multiplex_stage <- value from 0 - 7
// uint16_t seconds         <- value from 0 - 59
// returns bitfield 8 bits wide of current bitmap
uint32_t bitfield_data = 0;
uint16_t seconds_segment = 0;
uint16_t seconds_modulo = 0;
    seconds_segment = (seconds+1)/8; // Value 0 to 7
	seconds_modulo = (seconds+1)%8;  // Value 0 to 7

	if(multiplex_stage > 7)
	{
		return 0;
	}

	if(multiplex_stage < seconds_segment)  			// Quick determination of all 8 LEDs lit
	{
		bitfield_data = 0xFF;
	}
	else if(multiplex_stage > seconds_segment)  	// Quick determinaton of no LEDs lit
	{
		bitfield_data = 0x00;
	}
	else
	{
		bitfield_data = 0xFF00 >> seconds_modulo;   // shifts is a value 0-8
	}

	if(multiplex_stage == 0)
	{
	    bitfield_data = bitfield_data | 0b10000000; // Always illuminate "0" LED
	}

	bitfield_data = bitfield_data & 0xFF; // mask off other bits
	return bitfield_data;

// Note, "60" (State 7, bit 6) is always set!

}

//-------------------------------------------------------------------------
//  NAME
//      decimalToBCD
//
//  DESCRIPTION
//!     Convert decimal value to BCD
// ones_tens = OPT_BOTH, just returns BCD for both digits
// ones_tens = OPT_ONES, returns ones only
// ones_tens = OPT_TENS, returns tens only
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
uint16_t decimalToBCD(uint16_t decimal, uint16_t ones_tens)
{
    switch(ones_tens)
	{
	    case OPT_BOTH:
		{
		    return (((decimal/10) << 4) | (decimal % 10));
			break;
		}
		case OPT_ONES:
		{
		    return (decimal % 10);
			break;
		}
		case OPT_TENS:
		{
			return (decimal/10);
			break;
		}
		default:
		{
		    return 0;
			break;
		}

	}

}

//-------------------------------------------------------------------------
//  NAME
//      spi_pre_transfer_callback
//
//  DESCRIPTION
//!     Configure latches and output enable on 74HC595
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void spi_pre_transfer_callback(spi_transaction_t *t)
{
//	gpio_set_level(OUTPUT_ENABLE_INV, 1);				// Disable outputs
}

//-------------------------------------------------------------------------
//  NAME
//      spi_post_transfer_callback
//
//  DESCRIPTION
//!     Configure latches and output enable on 74HC595
//
//  RETURNS
//! @return void
//
//-------------------------------------------------------------------------
void spi_post_transfer_callback(spi_transaction_t *t)
{
	if(multiplex_anodeORcathode == REFRESH_DISABLE_ANODES)
	{
		gpio_set_level(LATCH_ANODES, 1);					// Latch outputs
		gpio_set_level(LATCH_ANODES, 0);
		multiplex_anodeORcathode = REFRESH_CATHODES;
	}
	else if(multiplex_anodeORcathode == REFRESH_CATHODES)
	{
		gpio_set_level(LATCH_CATHODES, 1);					// Latch outputs
		gpio_set_level(LATCH_CATHODES, 0);
		multiplex_anodeORcathode = REFRESH_ANODES;
	}
	else if (multiplex_anodeORcathode == REFRESH_ANODES)
	{
		gpio_set_level(LATCH_ANODES, 1);					// Latch outputs
		gpio_set_level(LATCH_ANODES, 0);
		multiplex_anodeORcathode = REFRESH_COMPLETE;
	}
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
	          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s",
                 WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
