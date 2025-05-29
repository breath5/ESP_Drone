
#include "LED.h"//自己的头文件

#include "Data_declaration.h"//全局变量存储地

#include <stdio.h>//兼容
#include <stdbool.h>//兼容
#include <unistd.h>//兼容

#include "freertos/FreeRTOS.h"//RTOS头文件
#include "freertos/task.h"    //RTOS头文件


void led_Task(void *pvParameter)
{
	bool temp_1 = false;

	while(1){
		if(init_ok){

			if(state.isRCLocked)gpio_set_level(LED1, 0);
			else gpio_set_level(LED1, 1);

			if(VBAT <= 380) {
				gpio_set_level(LED2, !temp_1);
			}
			else gpio_set_level(LED2, 0);

			vTaskDelay(pdMS_TO_TICKS(100));
		}

	}
}

void LED_init(void){


	gpio_config_t led_conf;

	led_conf.mode = GPIO_MODE_OUTPUT;		     // 配置gpio的模式
	led_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // 失能中断
	led_conf.pin_bit_mask = 1ULL << LED1;// 配置GPIO_IN寄存器，选择初始化的GPIO3口为led控制
	led_conf.pull_down_en = 0;                   // 下拉失能
	led_conf.pull_up_en = 0;                     // 上拉失能

	gpio_config(&led_conf);                       // 配置gpio参数，并使能

	led_conf.pin_bit_mask = 1ULL << LED2;// 配置GPIO_IN寄存器，选择初始化的GPIO3口为led控制
	gpio_config(&led_conf);                       // 配置gpio参数，并使能

	xTaskCreate(led_Task, "led_Task", 1024 * 5, NULL, 8, NULL);

}
// gpio的使用和单一模式一致
// 设置gpio3输出高电平：


