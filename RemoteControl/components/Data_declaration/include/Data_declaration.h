
#ifndef DATA_DECLARATION_H_
#define DATA_DECLARATION_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"


//PID算法的数据结构
typedef struct PID
{
  float P;         //参数
  float I;
  float D;
  float Error;     //比例项
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;
  float PrePreError;
  float Ilimit;
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;
  uint8_t Ilimit_flag;    //积分分离
}PID_TYPE;

typedef struct
{
	float pressure;
	float temperature;
	float asl;
} baro_t;

typedef struct
{
		int16_t X;
		int16_t Y;
		int16_t Z;

} INT16_XYZ;

typedef struct
{

		float X;
		float Y;
		float Z;

} FLOAT_XYZ;

typedef struct
{
	INT16_XYZ acc_n;		//加速度原始数值
	INT16_XYZ gyro_n;	//陀螺仪原始数值
	INT16_XYZ mag_n; 	//磁力计原始数值
	baro_t baro;	//气压计原始数值

	FLOAT_XYZ acc_f;		//加速度原始数值
	FLOAT_XYZ gyro_f;	//陀螺仪原始数值

} sensorData_t;

typedef struct
{
	uint32_t timestamp;	//时间戳

//	uint32_t roll;
//	uint32_t pitch;
//	uint32_t yaw;

	float roll;
    float pitch;
    float yaw;
} attitude_t;

// 四元数的定位
typedef struct quaternion_s
{
	uint32_t timestamp; //时间戳

	union
	{
		struct
		{
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct
		{
			float x;
			float y;
			float z;
			float w;
		};
	};
} quaternion_t;

struct  vec3_s
{
	uint32_t timestamp;	//时间戳

	float X;
	float Y;
	float Z;
};

typedef enum
{
	modeDisable = 0,/*关闭模式*/
	modeAbs,		/*绝对值模式*/
	modeVelocity	/*速率模式*/
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_b;

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct
{
	attitude_t attitude;		// deg
	attitude_t attitudeRate;	// deg/s
	point_t position;         	// m
	velocity_t velocity;      	// m/s
	mode_b mode;
	float AUX1;
	float AUX2;
	float AUX3;
	float AUX4;
	float AUX5;
	float AUX6;
	float thrust;
} setpoint_t;



typedef struct
{
	attitude_t attitude;//欧拉角
	//quaternion_t attitudeQuaternion;//四元数
	//point_t position; //位置
	//velocity_t velocity;//速度
	//acc_t acc; //角速度
	int airplane_mode; //飞行器当时模式
	bool isRCLocked;//遥控器锁定
} state_t;

extern bool init_ok;
extern uint32_t VBAT;
extern uint32_t UAV_VBAT;
extern uint32_t alt;
sensorData_t sensorData;//传感器原始数据
state_t state;          //状态
setpoint_t setpoint;    //遥控器


#endif /* COMPONENTS_DATA_DECLARATION_DATA_DECLARATION_H_ */
