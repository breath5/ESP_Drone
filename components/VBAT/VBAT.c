#include "VBAT.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "Data_declaration.h"

// ADC所接的通道  GPIO34 if ADC1  = ADC1_CHANNEL_6
#define ADC1_TEST_CHANNEL ADC1_CHANNEL_3  //GPIO4
// ADC斜率曲线
static esp_adc_cal_characteristics_t *adc_chars;
// 参考电压
#define DEFAULT_VREF				3300			//使用adc2_vref_to_gpio（）获得更好的估计值


void check_efuse(void)
{
    //检查TP是否烧入eFuse   是否支持两点校准
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //检查Vref是否烧入eFuse  是否支持Verf校准
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void VBAT_task(void *pvParameters){
	uint32_t read_raw;
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 100;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{
		read_raw = adc1_get_raw(ADC1_TEST_CHANNEL);// 采集ADC原始值//这里可以多次采样取平均值
		VBAT = esp_adc_cal_raw_to_voltage(read_raw, adc_chars);//通过一条斜率曲线把读取adc1_get_raw()的原始数值转变成了mV
		VBAT = VBAT*2/10;

		}
	}
}

void VBAT_init(void)
{
	check_efuse();
	adc1_config_width(ADC_WIDTH_BIT_12);// 12位分辨率
	adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));	// 为斜率曲线分配内存  为ADC校准特性结构体分配内存
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	xTaskCreate(VBAT_task, "VBAT_task", 1024*2, NULL, 0, NULL);//创建任务不断循环获取电池电压
}



