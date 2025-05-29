/* BSD AP+STA API Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.*/



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
#include "lwip/sys.h"
#include "lwip/ip4_addr.h"
#include "lwip/inet.h"
#include "lwip/lwip_napt.h"

#define AP_DEFAULT_SSID         "ESP-Drone"
#define AP_DEFAULT_PASSWORD     "kexie555good"
#define AP_DEFAULT_MAX_CONN     (5)

typedef struct {
    wifi_sta_config_t sta;
    wifi_ap_config_t ap;
} custom_wifi_config_t;

static custom_wifi_config_t custom_wifi_config = {
        .sta = {
                .ssid = "su", // target AP SSID
                .password = "12345678kexie-good", // target AP password
                .bssid_set = false,
        },
        .ap = {
                .ssid = AP_DEFAULT_SSID,
                .password = AP_DEFAULT_PASSWORD,
                .max_connection = AP_DEFAULT_MAX_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
};

static const char *TAG = "example";

static void custom_wifi_ap_staconnected_cb(void *arg, esp_event_base_t event_base,
                                           int32_t event_id, void *event_data) {
    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
    ESP_LOGI(TAG, "station connected, AID=%d", event->aid);
}

static void custom_wifi_ap_stadisconnected_cb(void *arg, esp_event_base_t event_base,
                                              int32_t event_id, void *event_data) {
    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
    ESP_LOGI(TAG, "station disconnected, AID=%d", event->aid);
}

static void custom_wifi_got_ip_cb(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Got IP event!");
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "IPv4 address: " IPSTR, IP2STR(&event->ip_info.ip));
}

static void custom_wifi_disconnect_cb(void *arg, esp_event_base_t event_base,
                                      int32_t event_id, void *event_data) {
    static int count = 5;
    esp_err_t err;
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    if (count > 0) {
        err = esp_wifi_connect();
        if (err == ESP_ERR_WIFI_NOT_STARTED) {
            return;
        }
        count--;
    } else {
        ESP_LOGE(TAG, "try max connect count, but failed");
    }
}

bool custom_wifi_ap_init() {
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_AP();
    esp_netif_t *ap_netif = esp_netif_new(&netif_config);
    assert(ap_netif);
    esp_netif_attach_wifi_ap(ap_netif);

    // Manually set IP address for AP
    esp_netif_ip_info_t ip_info = {};
    IP4_ADDR(&ip_info.ip, 192, 168, 43, 42);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    IP4_ADDR(&ip_info.gw, 192, 168, 43, 42);

    printf("DHCP server stop ");
    if (esp_netif_dhcps_stop(ap_netif) == ESP_OK) printf("success\n");
    else printf("fail\n");

    esp_err_t ret = esp_netif_set_ip_info(ap_netif, &ip_info);
    if (ret == ESP_OK) {
        printf("IP address set success\n");
        printf("IPV4 Address: 192.168.43.42\n");
        printf("IPV4 Netmask: 255.255.255.0\n");
        printf("IPV4 Gateway: 192.168.43.42\n");
    } else {
        printf("IP address set fail\n");
    }

    printf("DHCP server start ");
    if (esp_netif_dhcps_start(ap_netif) == ESP_OK) printf("success\n");
    else printf("fail\n");

    /********************添加NAPT**************************/
//    ip_addr_t dnsserver;
//    // Enable DNS (offer) for dhcp server
//    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
//    dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value));
//    // Set custom dns server address for dhcp server 默认跟随路由器 【推荐换成国内DNS】
//    dnsserver.u_addr.ip4.addr = htonl(0xC0A80301);
//    dnsserver.type = IPADDR_TYPE_V4;
//    dhcps_dns_setserver(&dnsserver);
//
//    u32_t napt_netif_ip = 0xC0A80401;
//    ip_napt_enable(htonl(napt_netif_ip), 1);



    /********************添加NAPT**************************/

    return true;
}

bool custom_wifi_sta_init(void) {
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
    esp_netif_t *netif = esp_netif_new(&netif_config);
    assert(netif);
    esp_netif_attach_wifi_station(netif);
    return true;
}

esp_err_t custom_ap_sta_init(void) {

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    custom_wifi_ap_init();
    custom_wifi_sta_init();

    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &custom_wifi_ap_staconnected_cb, NULL);
    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &custom_wifi_ap_stadisconnected_cb, NULL);

    esp_wifi_set_config(ESP_IF_WIFI_AP, (wifi_config_t*)&custom_wifi_config.ap);

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &custom_wifi_disconnect_cb, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &custom_wifi_got_ip_cb, NULL);

    esp_wifi_set_config(ESP_IF_WIFI_STA, (wifi_config_t*)&custom_wifi_config.sta);
    esp_wifi_connect();

    return ESP_OK;
}

