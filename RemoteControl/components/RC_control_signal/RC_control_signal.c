
#include "include/RC_control_signal.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <stdlib.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "Data_declaration.h"

// ADC所接的通道  GPIO34 if ADC1  = ADC1_CHANNEL_6 ROLL_CHANNEL
#define ROLL_CHANNEL 	ADC1_CHANNEL_6
#define PITCH_CHANNEL 	ADC1_CHANNEL_2
#define YAW_CHANNEL 	ADC1_CHANNEL_5
#define thrust_CHANNEL 	ADC1_CHANNEL_4

#define VBAT_CHANNEL 	ADC1_CHANNEL_3


#define button1 	(16)
#define button2 	(17)
#define button3 	(18)
#define button4 	(8)

// ADC斜率曲线
static esp_adc_cal_characteristics_t *adc_chars;
// 参考电压
#define DEFAULT_VREF				3300			//使用adc2_vref_to_gpio（）获得更好的估计值


void check_efuse(void)
{
    //检查TP是否烧入eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //检查Vref是否烧入eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


//将输入值从一个范围映射到另一个范围
float map(float val,int in_min,int in_max,int out_min,int out_max){
	float out =  ((float)val - (float)in_min) /(((float)in_max-(float)in_min)/((float)out_max-(float)out_min))+(float)out_min;
	return out;
}
//在数组中找到一个元素的位置
float FindPos(float*a,int low,int high){ //确定一个元素的位序
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
        {
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]
        }
        while(low<high && a[low]<=val)
        {
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
        }
    }
    a[low] = val;
    return low;
}
//快速排序算法，用于对数组进行排序。
void QuiteSort(float* a,int low,int high){ //快速排序
     int pos;
     if(low<high)
     {
         pos = FindPos(a,low,high); //排序一个位置
         QuiteSort(a,low,pos-1);    //递归调用
         QuiteSort(a,pos+1,high);
     }
 }
//窗口滑动滤波器函数
void  filter(float acc,float Acc_filt,uint8_t n){ //窗口滑动滤波函数
  static float buf[20];
  static uint8_t cnt =0,flag = 1;
  Acc_filt = 0;
  float temp1=Acc_filt;
  uint8_t i;
  buf[cnt] = acc;
  cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
  if(cnt<n && flag) //如果两个条件都不为零则条件满足
    return;   //数组填不满不计算
  else
    flag = 0;

  QuiteSort(buf,0,n-1);
  for(i=1;i<n-1;i++)
   {
    temp1 += buf[i];
   }

   if(cnt>=n) cnt = 0;

   Acc_filt  = (float)temp1/(n-2);

}


//获取遥杆ADC原始值并对其进行滤波的任务函数
static void control_signal_task(void *pvParameters){ //获取遥杆ADC原始值滤波并处理
	int isRCLocked_ent_on = 0;
	int isRCLocked_ent_off = 0;
	bool temp_1 = false;

	uint32_t read_raw;
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 5;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{



		float roll =(float)adc1_get_raw(ROLL_CHANNEL);//获取横滚值并进行滑动滤波
		float pitch = (float)adc1_get_raw(PITCH_CHANNEL);//获取俯仰值并进行滑动滤波
		float yaw = (float)adc1_get_raw(YAW_CHANNEL);//获取航向值并进行滑动滤波
		float thrust =(float)adc1_get_raw(thrust_CHANNEL);//获取油门值并进行滑动滤波

		filter(roll,roll,10);//对加速度原始数据进行去极值滑动窗口滤波

		read_raw = adc1_get_raw(VBAT_CHANNEL);// 采集ADC原始值//这里可以多次采样取平均值
		uint32_t voltage = esp_adc_cal_raw_to_voltage(read_raw, adc_chars);//通过一条斜率曲线把读取adc1_get_raw()的原始数值转变成了mV
        VBAT = voltage*2/10;

		roll = map(roll,400,3750,0,3000); //等比映射成需要的区间
		pitch = map(pitch,400,3750,0,3000); //等比映射成需要的区间
		yaw = map(yaw,400,3750,0,3000); //等比映射成需要的区间
		thrust = map(thrust,400,3750,0,1000); //等比映射成需要的区间
		//限制值的范围
		if(roll   < 0  ) 	roll   = 0;
		if(pitch  < 0  ) 	pitch  = 0;
		if(yaw    < 0  ) 	yaw    = 0;
		if(thrust < 0  ) 	thrust = 0;
		if(thrust >1000)    thrust = 1000;

		 setpoint.attitude.roll = roll;
		 setpoint.attitude.pitch = pitch;
		 setpoint.attitude.yaw = yaw;
         setpoint.thrust = thrust + 35;
//            setpoint.attitude.yaw = 1500;
//            setpoint.attitude.roll = 1500;
//            setpoint.attitude.pitch = 1500;
//		    setpoint.thrust = 500;

		 setpoint.AUX1 = gpio_get_level(button1);
		 setpoint.AUX2 = gpio_get_level(button2);
		 setpoint.AUX3 = gpio_get_level(button3);
		 setpoint.AUX4 = gpio_get_level(button4);

		 if(temp_1 == true && setpoint.AUX1 == 0){ //捕获上升沿
			 state.isRCLocked  = !state.isRCLocked; //取反信号
		 }
		 temp_1 = (bool)setpoint.AUX1;

		}
	}
}
//初始化ADC、GPIO和任务的函数
void control_signal_init(void)
{
	  check_efuse();
	  adc1_config_width(ADC_WIDTH_BIT_12);// 12位分辨率
	  adc1_config_channel_atten(ROLL_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	  adc1_config_channel_atten(PITCH_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	  adc1_config_channel_atten(YAW_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	  adc1_config_channel_atten(thrust_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	  adc1_config_channel_atten(VBAT_CHANNEL, ADC_ATTEN_DB_11);// 电压输入衰减
	  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));	// 为斜率曲线分配内存

	  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

	  //初始化输入按钮
	  gpio_config_t button_conf;
	  button_conf.mode = GPIO_MODE_INPUT;		         // 配置gpio的为输入模式
	  button_conf.intr_type = GPIO_PIN_INTR_DISABLE;     // 禁用中断
	  button_conf.pin_bit_mask = 1ULL << button1;        // 配置GPIO_IN寄存器，选择初始化的GPIO3口为led控制
	  button_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;  // 禁用下拉电阻
	  button_conf.pull_up_en = GPIO_PULLUP_ENABLE;       // 启用上拉电阻

	  gpio_config(&button_conf);                         // 配置gpio参数，并使能

	  button_conf.pin_bit_mask = 1ULL << button2;        // 配置GPIO_IN寄存器，选择初始化的引脚
	  gpio_config(&button_conf);                         // 配置gpio参数，并使能

	  button_conf.pin_bit_mask = 1ULL << button3;        // 配置GPIO_IN寄存器，选择初始化的引脚
	  gpio_config(&button_conf);                         // 配置gpio参数，并使能

	  button_conf.pin_bit_mask = 1ULL << button4;        // 配置GPIO_IN寄存器，选择初始化的引脚
	  gpio_config(&button_conf);                         // 配置gpio参数，并使能

      xTaskCreate(control_signal_task, "control_signal_task", 1024*4, NULL, 3, NULL);  //启用信号获取任务，循环获取摇杆与按钮的信号
}

