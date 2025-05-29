
#include "control.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <sys/time.h>

#include <inttypes.h>

#include "MPU6050.h"
#include "IMU.h"
#include "Data_declaration.h"

#include "PID.h"
#include "IMU.h"
#include "PWM.h"

#include "SPL06-001.h"

struct timeval tv_now;
int64_t time1_us = 0;
int64_t time_us = 0;



float Pre_THROTTLE,THROTTLE;
float Moto_PWM_1=0.0f,Moto_PWM_2=0.0f,Moto_PWM_3=0.0f,Moto_PWM_4=0.0f;

int mun = 0;

float rc_yaw = 0;
float control_yaw = 0;
bool lock_p = false;

void Control(state_t *att_in,sensorData_t *gyr_in, setpoint_t *rc_in)
{
	attitude_t Measure_Angle,Target_Angle;
	Measure_Angle.roll = att_in->attitude.roll;
	Measure_Angle.pitch = att_in->attitude.pitch;
	Measure_Angle.yaw = att_in->attitude.yaw - control_yaw;
	Target_Angle.roll = (float)((rc_in->attitude.roll-1500)/35.0f);
	Target_Angle.pitch = (float)((rc_in->attitude.pitch-1500)/35.0f);   //数值映射
	Target_Angle.yaw = (float)((1500-rc_in->attitude.yaw)/35.0f);
    if(Target_Angle.yaw >1 || Target_Angle.yaw <-1) rc_yaw -= Target_Angle.yaw/100;

	//角度环
	PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.roll,Measure_Angle.roll);//ROLL角度环PID （输入角度 输出角速度）
	PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pitch,Measure_Angle.pitch);//PITH角度环PID （输入角度 输出角速度）
    PID_Postion_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);//YAW角度环PID  （输入角度 输出角速度）

	//角速度环
	PID_Postion_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,(gyr_in->gyro_f.Y*RadtoDeg)); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,(gyr_in->gyro_f.X*RadtoDeg)); //PITH角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_YAW_Rate,PID_YAW_Angle.OutPut,(gyr_in->gyro_f.Z*RadtoDeg)); //YAW角速度环PID （输入角度，输出电机控制量）

	//动力分配
	if(att_in->isRCLocked)
	{
//		if(lock_p == false) {
//			control_yaw = att_in->attitude.yaw ; //解锁时清空yaw值
//			rc_yaw = 0;
//		}

		if(rc_in->thrust>20){
			Moto_PWM_1 = rc_in->thrust + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_2 = rc_in->thrust - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
			Moto_PWM_3 = rc_in->thrust - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_4 = rc_in->thrust + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
		}else{
			  mun++;
		      if (mun >= 1)Moto_PWM_1 = 5;
		      if (mun >= 10)Moto_PWM_2 = 5;
		      if (mun >= 20)Moto_PWM_3 = 5;
		      if (mun >= 30)Moto_PWM_4 = 5;
		      if (mun >= 40)mun = 40;
		}
	}
	else
	{
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
		mun = 0;
	}

   Moto_Pwm(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4); //将此数值分配到定时器，输出对应占空比的PWM波

}



void angle_control_Task(void *pvParameter)
{

	const TickType_t adg = 10;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	TickType_t adp = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{

			Height_Get(); // 取得高度
			mpu6050_read_data(0x6B,&sensorData);
			Prepare_Data(&sensorData); //获取mpu6050原始数值，滤波并转换单位
			IMUupdate(&sensorData,&state);//将姿态数据通过四元数转为欧拉角
			Control(&state,&sensorData,&setpoint);



		}
	}
}
void control_init(void){
	xTaskCreate(angle_control_Task, "angle_control_Task", 1024 * 8, NULL, 24, NULL);
}


