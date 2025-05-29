#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "remote_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Data_declaration.h"
#include "SPI.h"

#include "WIFI.h"
#include "UDP_TCP.h"
#include "LED.h"

#include "RC_control_signal.h"
#include "lv_task.h"

void app_main(void)
{

	wifi_init_sta();//连接WiFi
	vTaskDelay(pdMS_TO_TICKS(1000));//等待WiFi连接
	udp_client_init();//UDP初始化
	remote_init(); //遥控器通信初始化

	LED_init(); //LED灯初始化
	control_signal_init(); //ADC输入初始化
    lv_task_init();             // LVGL屏幕驱动初始化

	init_ok = true;//初始化完成置1让所有任务开始运行

}
