
#include "remote_control.h"
#include "Data_declaration.h"

#include "esp_log.h"
#include "UDP_TCP.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "UART.h"

#define DATA_write  udp_rc_send

uint8_t data_to_send[50];	//发送数据缓存

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void rc_Send_data(void) //遥控器数据发送
{//格式仿匿名上位机
	uint8_t _cnt=0; //发送缓冲区字节数
	uint8_t sum = 0;//校验
	uint16_t _temp;//临时变量1


	data_to_send[_cnt++]=0xBB;//4个B的帧头
	data_to_send[_cnt++]=0xBB;//4个B的帧头
	data_to_send[_cnt++]=0x01;//功能字为01
	data_to_send[_cnt++]=0;	  //这个是数据长度现在先不填后面再填

	_temp = setpoint.attitude.roll;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.attitude.pitch;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.attitude.yaw;
//    _temp = 1500;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.thrust;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX3;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX4;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX5;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp = setpoint.AUX6;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[_cnt++]=state.airplane_mode;//飞行器模式

	data_to_send[_cnt++]=state.isRCLocked;//飞行是否锁定

	data_to_send[3] = _cnt-4; //_cnt累计的长度减去前面的4个非数据字节等于数据的长度

	for(int i = 0 ; i < _cnt ; i++) sum += data_to_send[i];

	data_to_send[_cnt++]=sum;
	//printf("sum:%d\n",sum);
	//printf("locked=%d  \n",(int)state.isRCLocked);

	udp_rc_send(data_to_send, _cnt);
    //uart_write_bytes(UART_NUM_0, data_to_send, _cnt);
}

static void remote_send_task(void *pvParameters){
	uint32_t read_raw;
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 50;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{
			rc_Send_data();
		}
	}
}




void remote_init(void){
	xTaskCreate(remote_send_task, "remote_send_task", 1024*2, NULL, 3, NULL);
}
