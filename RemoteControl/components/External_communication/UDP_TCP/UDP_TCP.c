#include "UDP_TCP.h"

#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "Data_declaration.h"

#include <sys/param.h>
#include "esp_netif.h"


#include "UDP_TCP.h"

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#define HOST_IP_ADDR "192.168.43.42"

#define PORT 5555

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

char rx_buffer[128];
char host_ip[] = HOST_IP_ADDR;
int addr_family = 0;
int ip_protocol = 0;
int sock;
struct sockaddr_in dest_addr;
socklen_t socklen_rc = sizeof(dest_addr);//计算遥控器存储变量大小


typedef union
{
    float float_value;
    uint8_t uint8[4];
}packet_uint32_to_float;

float uint32_to_float(uint16_t value1, uint16_t value2)
{
    packet_uint32_to_float packet;

    uint8_t buf[4]; memset(buf,0x00,sizeof(buf));
    buf[0] = (uint8_t)((value1 >> 8) & 0xFF);
    buf[1] = (uint8_t)((value1) & 0xFF);
    buf[2] = (uint8_t)((value2 >> 8) & 0xFF);
    buf[3] = (uint8_t)((value2) & 0xFF);
    uint32_t uint32 = ((buf[0]<<24) & 0XFFFFFFFF) + ((buf[1]<<16) & 0XFFFFFF) + ((buf[2]<<8) & 0XFFFF) + buf[3];

    for(uint8_t i = 0; i < 4; i++)
    {
        packet.uint8[i] = (uint8_t)(uint32>>(i*8));
    }
    printf("modbus_uint32_to_float  uint32=%d, value1=%d; value2=%d; float_value = %f;\n",uint32,value1,value2,packet.float_value);
    return packet.float_value;
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


void rc_data_decode (char *data){
	uint8_t sum = 0; //校验
	uint32_t _temp; //临时变量

	if(data[0] != 0xBB && data[1] != 0xBB)  return; //当帧头不等于4个B就退出舍弃这段数据
	for(int i = 0;i < data[3] + 4 ;i++) sum+=(uint8_t)data[i];
	if(sum != data[data[3]+4]) return; //当校验不等时就退出舍弃这段数据
	//printf("数据校验正确\n");

    _temp = ((uint8_t)data[4]<< 24) + ((uint8_t)data[5] << 16) + ((uint8_t)data[6]  << 8) + (uint8_t)data[7];
    state.attitude.roll = U32ToFloat(_temp);


    _temp = ((uint8_t)data[8]<< 24) + ((uint8_t)data[9] << 16) + ((uint8_t)data[10]  << 8) + (uint8_t)data[11];
    state.attitude.pitch = U32ToFloat(_temp);

    _temp = ((uint8_t)data[12]<< 24) + ((uint8_t)data[13] << 16) + ((uint8_t)data[14]  << 8) + (uint8_t)data[15];
    state.attitude.yaw = U32ToFloat(_temp);

	alt = ((uint8_t)data[16]<< 24) + ((uint8_t)data[17] << 16) + ((uint8_t)data[18]  << 8) + (uint8_t)data[19];

	UAV_VBAT = ((uint8_t)data[20]<< 24) + ((uint8_t)data[21] << 16) + ((uint8_t)data[22]  << 8) + (uint8_t)data[23];


}


static void udp_client_task(void *pvParameters)
{
    while (1) {
        while (1) {
        	struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        	            socklen_t socklen = sizeof(source_addr);
        	            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &socklen_rc);
        	            if (len < 0) {
        	                printf("接收失败\n");
        	                break;
        	            }
        	            // Data received
        	            else {
        	            	rc_data_decode(rx_buffer);

        	                  }

        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}




void udp_rc_send(uint8_t *data,uint8_t len)
{

    int err = sendto(sock, data, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

}


void udp_client_init(void){


    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    xTaskCreate(udp_client_task, "udp_client", 4096*2, NULL, 5, NULL);

}


