#include "remote_control.h"
#include "Data_declaration.h"
#include "UART.h"

#include "esp_log.h"
#include "UDP_TCP.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint8_t data_to_send[50];	//发送数据缓存

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


void rc_data_decode (char *data){
	uint8_t sum = 0; //校验
	uint16_t _temp; //临时变量

	if(data[0] != 0xBB && data[1] != 0xBB)  return; //当帧头不等于4个B就退出舍弃这段数据
	for(int i = 0;i < data[3] + 4 ;i++) sum+=(uint8_t)data[i];
	if(sum != data[data[3]+4]) return; //当校验不等时就退出舍弃这段数据
	//printf("数据校验正确\n");
	state.rc_link = true; //确认连接信号刷新

	_temp = (uint8_t)data[4]  << 8 | (uint8_t)data[5];
	setpoint.attitude.roll =  _temp + roll_trim;

	_temp = (uint8_t)data[6]  << 8 | (uint8_t)data[7];
	setpoint.attitude.pitch = (1500 - ( _temp - 1500)) + pitch_trim;

	_temp = (uint8_t)data[8]  << 8 | (uint8_t)data[9];
	setpoint.attitude.yaw =   _temp + yaw_trim;

	_temp = (uint8_t)data[10]  << 8 | (uint8_t)data[11];
	setpoint.thrust =  _temp;

	//_temp = (uint8_t)data[12]  << 8 | (uint8_t)data[13];
	//setpoint.AUX1 =  _temp;

	if(data[25] == 1 && isRCLocked_p == 0) state.isRCLocked = true; //捕获上升沿 当遥控器的数据从0变成1时才解锁

	if(data[25] == 0) state.isRCLocked = false;

	isRCLocked_p = data[25];


	 //printf("ROLL=%d  ",(int)setpoint.attitude.roll);
	 //printf("PITCH=%d  ",(int)setpoint.attitude.pitch);
	 //printf("YAW=%d  ",(int)setpoint.attitude.yaw);
	 //printf("THRUST=%d  \n",(int)setpoint.thrust);
	 //printf("locked=%d  \n",state.isRCLocked);
	 //printf("locked=%d  \n",data[25]);

}

uint32_t FloatToU32(float dat)
{
	uint8_t buf[4];

	buf[0] = ((uint8_t*)&dat)[0];
	buf[1] = ((uint8_t*)&dat)[1];
	buf[2] = ((uint8_t*)&dat)[2];
	buf[3] = ((uint8_t*)&dat)[3];

	return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

float U32ToFloat(uint32_t dat)
{
	uint8_t buf[4];

	buf[0] = dat >> 24;
	buf[1] = dat >> 16;
	buf[2] = dat >> 8;
	buf[3] = dat & 0xff;

	return *((float*)buf);
}

//向遥控器发送姿态数据
void RC_Send_Status(float angle_rol, float angle_pit, float angle_yaw, float alt, uint32_t vbat)
{
	uint8_t _cnt=0; //发送缓冲区字节数
		uint8_t sum = 0;//校验
		uint32_t _temp;//临时变量1

		data_to_send[_cnt++]=0xBB;//4个A的帧头
		data_to_send[_cnt++]=0xBB;//4个A的帧头
		data_to_send[_cnt++]=0x01;//功能字为01
		data_to_send[_cnt++]=0;	  //这个是数据长度现在先不填后面再填

		_temp = FloatToU32(angle_rol);//横滚角x100转为int16类型
		data_to_send[_cnt++]=BYTE3(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE2(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE1(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE0(_temp);//int16低位填充

		_temp = FloatToU32(angle_pit);//横滚角x100转为int16类型
		data_to_send[_cnt++]=BYTE3(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE2(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE1(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE0(_temp);//int16低位填充

		_temp = FloatToU32(angle_yaw);//横滚角x100转为int16类型
		data_to_send[_cnt++]=BYTE3(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE2(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE1(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE0(_temp);//int16低位填充

		_temp = FloatToU32(alt);//横滚角x100转为int16类型
		data_to_send[_cnt++]=BYTE3(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE2(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE1(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE0(_temp);//int16低位填充

		_temp = vbat;//横滚角x100转为int16类型
		data_to_send[_cnt++]=BYTE3(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE2(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE1(_temp);//int16高位填充
		data_to_send[_cnt++]=BYTE0(_temp);//int16低位填充

		data_to_send[3] = _cnt-4; //_cnt累计的长度减去前面的4个非数据字节等于数据的长度

		for(int i = 0 ; i < _cnt ; i++) sum += data_to_send[i];

		data_to_send[_cnt++]=sum;

		UDP_write_rc(data_to_send, _cnt);
	//printf("发送成功\n");
}

void remote_control_task(void * pvParameters){//匿名上位机数据发送
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 10;
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
				{
			     RC_Send_Status(state.attitude.roll,state.attitude.pitch,state.attitude.yaw,ALT_BRO.h,VBAT);
				}
	}
}

void remote_control_init(void){
	xTaskCreate(remote_control_task,"remote_control_task",4096,NULL,6,NULL);//循环向遥控器反馈数据
}
