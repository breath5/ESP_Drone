/**
 * @file st7796s.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "st7796s.h"
#include "disp_spi.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "ST7796S"

/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct
{
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7796s_set_orientation(uint8_t orientation);

static void st7796s_send_cmd(uint8_t cmd);
static void st7796s_send_data(void *data, uint16_t length);
static void st7796s_send_color(void *data, uint16_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void st7796s_init(void)
{
	lcd_init_cmd_t init_cmds[] = {
		{0x11, {0x00}, 0},
		{0xF0, {0xC3}, 1},
		{0xF0, {0x96}, 1},
		{0x36, {0x28}, 1},
		{0x3A, {0x55}, 1},
		{0xB4, {0x01}, 1},
		{0xB7, {0xC6}, 1},
		{0xE8, {0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33}, 8},
		{0xC1, {0x06}, 1},
		{0xC2, {0xA7}, 1},
		{0xC5, {0x18}, 1},
		{0xE0, {0xf0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F, 0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B}, 14},
		{0xE1, {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2D, 0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B}, 15},
		{0xF0, {0x3C}, 1},
		{0xF0, {0x69}, 1},
		{0, {0}, 0xff},
	};

	//Initialize non-SPI GPIOs
	gpio_pad_select_gpio(ST7796S_DC);
	gpio_set_direction(ST7796S_DC, GPIO_MODE_OUTPUT);

#if ST7796S_USE_RST
	gpio_pad_select_gpio(ST7796S_RST);
	gpio_set_direction(ST7796S_RST, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ST7796S_RST, 0);
	vTaskDelay(100 / portTICK_RATE_MS);
	gpio_set_level(ST7796S_RST, 1);
	vTaskDelay(100 / portTICK_RATE_MS);
#endif

	ESP_LOGI(TAG, "Initialization.");

	//Send all the commands
	uint16_t cmd = 0;
	while (init_cmds[cmd].databytes != 0xff)
	{
		st7796s_send_cmd(init_cmds[cmd].cmd);
		st7796s_send_data(init_cmds[cmd].data, init_cmds[cmd].databytes & 0x1F);
		if (init_cmds[cmd].databytes & 0x80)
		{
			vTaskDelay(100 / portTICK_RATE_MS);
		}
		cmd++;
	}

	st7796s_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);

#if ST7796S_INVERT_COLORS == 1
	st7796s_send_cmd(0x21);
#else
	st7796s_send_cmd(0x20);
#endif
}

void st7796s_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
	uint8_t data[4];

	/*Column addresses*/
	st7796s_send_cmd(0x2A);
	data[0] = (area->x1 >> 8) & 0xFF;
	data[1] = area->x1 & 0xFF;
	data[2] = (area->x2 >> 8) & 0xFF;
	data[3] = area->x2 & 0xFF;
	st7796s_send_data(data, 4);

	/*Page addresses*/
	st7796s_send_cmd(0x2B);
	data[0] = (area->y1 >> 8) & 0xFF;
	data[1] = area->y1 & 0xFF;
	data[2] = (area->y2 >> 8) & 0xFF;
	data[3] = area->y2 & 0xFF;
	st7796s_send_data(data, 4);

	/*Memory write*/
	st7796s_send_cmd(0x2C);

	uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

	st7796s_send_color((void *)color_map, size * 2);
}

void st7796s_sleep_in()
{
	uint8_t data[] = {0x08};
	st7796s_send_cmd(0x10);
	st7796s_send_data(&data, 1);
}

void st7796s_sleep_out()
{
	uint8_t data[] = {0x08};
	st7796s_send_cmd(0x11);
	st7796s_send_data(&data, 1);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void st7796s_send_cmd(uint8_t cmd)
{
	disp_wait_for_pending_transactions();
	gpio_set_level(ST7796S_DC, 0); /*Command mode*/
	disp_spi_send_data(&cmd, 1);
}

static void st7796s_send_data(void *data, uint16_t length)
{
	disp_wait_for_pending_transactions();
	gpio_set_level(ST7796S_DC, 1); /*Data mode*/
	disp_spi_send_data(data, length);
}

static void st7796s_send_color(void *data, uint16_t length)
{
	disp_wait_for_pending_transactions();
	gpio_set_level(ST7796S_DC, 1); /*Data mode*/
	disp_spi_send_colors(data, length);
}

static void st7796s_set_orientation(uint8_t orientation)
{
	// ESP_ASSERT(orientation < 4);

	const char *orientation_str[] = {
		"PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"};

	ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

#if defined CONFIG_LV_PREDEFINED_DISPLAY_M5STACK
	uint8_t data[] = {0x68, 0x68, 0x08, 0x08};
#elif defined(CONFIG_LV_PREDEFINED_DISPLAY_WROVER4)
	uint8_t data[] = {0x4C, 0x88, 0x28, 0xE8};
#elif defined(CONFIG_LV_PREDEFINED_DISPLAY_WT32_SC01)
	uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};
#elif defined(CONFIG_LV_PREDEFINED_DISPLAY_NONE)
	uint8_t data[] = {0x48, 0x88, 0x28, 0xE8};
#endif

	ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

	st7796s_send_cmd(0x36);
	st7796s_send_data((void *)&data[orientation], 1);

	vTaskDelay(pdMS_TO_TICKS(80));
	st7796s_send_cmd(0x29);

	char sned_data_temp = 0x24;
	st7796s_send_cmd(0x53);
	st7796s_send_data((void *)&sned_data_temp, 1);

	sned_data_temp = 0xff;
	st7796s_send_cmd(0x51);
	st7796s_send_data((void *)&sned_data_temp, 1);
}
