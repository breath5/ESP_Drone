
#include "control.h"
#include "UART.h"
#include "MPU6050.h"
#include "IMU.h"
#include "anotc.h"
#include "LED.h"
#include "driver/gpio.h"
#include "WIFI.h"
#include "UDP_TCP.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "anotc.h"
#include <sys/time.h>
#include <inttypes.h>
#include "Data_declaration.h"
#include "remote_control.h"
#include "PWM.h"
#include "VBAT.h"
#include "SPI.h"
#include "SPL06-001.h"
#include "mqtt_lot.h"

void app_main(void)
{
	//uart1_init(115200);//初始化串口波特率115200
	IIC_init();//IIC初始化
	PWM_init();//PWM初始化
	VBAT_init();//电池电压检测初始化

	mpu6050_Init();//MPU6050姿态传感器初始化
	LED_init();//LED灯初始化

	wifi_init_ap();
    // custom_ap_sta_init();//初始化WiFi

	UDP_init();//UDP通信初始化
	anotc_Init();//匿名上位机初始化
	remote_control_init();//遥控器初始化

	control_init();//姿态控制初始化
    mqtt_app_start();  //飞行数据上报
	printf("剩余空闲堆栈: %d 字节\n", xPortGetFreeHeapSize());

	init_ok = true;//初始化完成置1让所有任务开始运行


}
