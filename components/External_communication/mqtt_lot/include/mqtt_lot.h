#pragma once


#ifndef __MQTT_LOT_
#define __MQTT_LOT_


#define  iot_clientID           "66eb880b617f154719224ee4_0000_0_0_2024092007"                        //客户端ID
#define  AIoT_SUBSCRIBE_TOPIC   "$oc/devices/66eb880b617f154719224ee4_0000/sys/properties/report"     //订阅主题
#define  iot_hostName           "fc6a8a10fd.iot-mqtts.cn-north-4.myhuaweicloud.com"   //服务器地址
#define  iot_userName           "66eb880b617f154719224ee4_0000"                       //客户端用户名
#define  iotda_pswd             "84b4ca6df6573bf483b16d2392d78e16249979d809d435d1ee68bc4a456e9688"  //客户端登入密码

void mqtt_app_start(void);


#endif 