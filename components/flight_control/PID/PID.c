
#include "PID.h"

#include "Data_declaration.h"

/*****************************************************************************
* 函  数：void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
* 功  能：位置式PID算法
* 参  数：PID: 算法P I D参数的结构体
*         target: 目标值
*         measure: 测量值
* 返回值：无
* 备  注: 角度环和角速度环共用此函数
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{

	PID->Error  = target - measure;              //误差
	PID->Differ = PID->Error - PID->PreError;    //微分量

	PID->Pout = PID->P * PID->Error;                        //比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
	PID->Dout = PID->D * PID->Differ;                       //微分控制

	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制

	if(state.isRCLocked == true && setpoint.thrust >= 180)    //飞机解锁之后再加入积分,防止积分过调
		{
			if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //积分分离
			{PID->Ilimit_flag = 0;}
			else
			{
				PID->Ilimit_flag = 1;                               //加入积分(只有测量值在-PID->Ilimit~PID->Ilimit 范围内时才加入积分)
				PID->Integral += PID->Error;                        //对误差进行积分
				if(PID->Integral > PID->Irang)                      //积分限幅
					PID->Integral = PID->Irang;
				if(PID->Integral < -PID->Irang)                     //积分限幅
				    PID->Integral = -PID->Irang;
			}
		}else
		{PID->Integral = 0;}

	PID->PreError = PID->Error ;                            //前一个误差值
}

void PidParameter_init(void)
{
//原始数值
//	roll_trim = 200;  //横滚角遥控数值补偿
//	pitch_trim = 0; //俯仰角遥控数值补偿
//	yaw_trim = 0;   //航向角遥控数值补偿

    roll_trim = 47;   //横滚角遥控数值补偿 P
    pitch_trim = -44; //俯仰角遥控数值补偿 I
    yaw_trim = 0;     //航向角遥控数值补偿 D

  //ROLL轴
  PID_ROL_Rate.Ilimit_flag = 0; //Roll轴角速度积分的分离标志
  PID_ROL_Rate.Ilimit = 150;    //Roll轴角速度积分范围
  PID_ROL_Rate.Irang = 1200;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  PID_ROL_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
  PID_ROL_Angle.Ilimit = 35;    //Roll轴角度积分范围
  PID_ROL_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

  //PITCH轴
  PID_PIT_Rate.Ilimit_flag = 0; //Pitch轴角速度积分的分离标志
  PID_PIT_Rate.Ilimit = 150;    //Pitch轴角速度积分范围
  PID_PIT_Rate.Irang = 1200;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  PID_PIT_Angle.Ilimit_flag = 0;//Roll轴角度积分的分离标志
  PID_PIT_Angle.Ilimit = 35;    //Roll轴角度积分范围
  PID_PIT_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

  //PID_YAW_Rate.Ilimit_flag = 0; //Yaw轴角速度积分的分离标志
  PID_YAW_Rate.Ilimit = 150;    //Yaw轴角速度积分范围
  PID_YAW_Rate.Irang = 1200;    //Yaw轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
  //PID_YAW_Angle.Ilimit_flag = 0;//Yaw轴角度积分的分离标志
  PID_YAW_Angle.Ilimit = 35;    //Yaw轴角度积分范围
  PID_YAW_Angle.Irang = 200;    //Yaw轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）


  //角速度环PID参数
  PID_ROL_Rate.P = 1.700;
  PID_ROL_Rate.I = 0.020;
  PID_ROL_Rate.D = 0.600f;
  PID_PIT_Rate.P = 1.700;
  PID_PIT_Rate.I = 0.020f;
  PID_PIT_Rate.D = 0.600f;
  PID_YAW_Rate.P = 5.000f;
  PID_YAW_Rate.I = 0.200f;
  PID_YAW_Rate.D = 0.050f;

  //角度环数据PID参数
  PID_ROL_Angle.P = 1.500f;
  PID_ROL_Angle.I = 0.020f;
  PID_ROL_Angle.D = 0.050f;
  PID_PIT_Angle.P = 1.500f;
  PID_PIT_Angle.I = 0.020f;
  PID_PIT_Angle.D = 0.050f;
  PID_YAW_Angle.P = 0.200f;
  PID_YAW_Angle.I = 0.020f;
  PID_YAW_Angle.D = 0.000f;

}


