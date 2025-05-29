/*
 * Data declaration.c
 *  数据声明，结构体，堆栈空间，飞行器姿态和PID各类数据在此声明
 */

#include "Data_declaration.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"

bool init_ok = false;//初始化成功后置1全部任务才会开始运行
int64_t task_run_time_us[10] = {0};  //系统运行时间
int roll_trim;  //横滚角遥控数值补偿
int pitch_trim; //俯仰角遥控数值补偿
int yaw_trim;   //航向角遥控数值补偿
int Acceleration_calibration;
int gyroscope_calibration;
int AirPressure_calibration;
int isRCLocked_p = 0;
uint32_t VBAT;  //电压

