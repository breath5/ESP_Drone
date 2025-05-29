#include "mqtt_client.h"
#include "esp_log.h"
#include "mqtt_lot.h"
#include "cJSON.h"
#include "Data_declaration.h"

static const char *TAG = "MQTT";

static void mqtt_pulish_task(void *client) {
    printf("Entering mqtt_pulish_task\n");
    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t)client;

    // 预先创建不变的 cJSON 对象
    cJSON *root = cJSON_CreateObject();
    cJSON *services = cJSON_CreateArray();
    cJSON *service = cJSON_CreateObject();
    cJSON *properties = cJSON_CreateObject();

    // 构建不变的属性值
    cJSON_AddItemToObject(root, "services", services);
    cJSON_AddItemToArray(services, service);
    cJSON_AddStringToObject(service, "service_id", "flight_data");
    cJSON_AddItemToObject(service, "properties", properties);
    int x = 4643;

    while (1) {
        // 更新动态数值


        // 清空 properties 对象中的内容
        cJSON_DeleteItemFromObject(service, "properties");
        properties = cJSON_CreateObject();
        cJSON_AddItemToObject(service, "properties", properties);

        // 添加数值到 properties 对象

        cJSON_AddNumberToObject(properties, "Roll", state.attitude.roll);
        cJSON_AddNumberToObject(properties, "Pitch",  state.attitude.pitch);
        cJSON_AddNumberToObject(properties, "Yaw", state.attitude.yaw);
        cJSON_AddNumberToObject(properties, "Thrust", setpoint.thrust);


        // 将 JSON 数据转为字符串
        const char *json_string = cJSON_PrintUnformatted(root);

        // 向主题发布消息
        int msg_id = esp_mqtt_client_publish(mqtt_client, AIoT_SUBSCRIBE_TOPIC, json_string, 0, 1, 0);
        ESP_LOGI(TAG, "Published JSON message: %s, msg_id=%d", json_string, msg_id);

        // 释放 JSON 字符串
        free((void *)json_string);
        // 可选择性延迟发布频率，避免过快发布
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 释放 JSON 对象
    cJSON_Delete(root);

    // 删除任务
    vTaskDelete(NULL);
}




//函数可以用来记录非零的错误代码，并在需要时提供有用的调试信息。
static void log_error_if_nonzero(const char * message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGI(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

//mqtt事件处理函数
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event){
   esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch(event->event_id){

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        //在此处订阅主题，或取消订阅主题
        msg_id = esp_mqtt_client_subscribe(client, AIoT_SUBSCRIBE_TOPIC, 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        xTaskCreate(mqtt_pulish_task, "mqtt_pulish_task", 1024*4, client, 5, NULL);
        break;

    case MQTT_EVENT_DISCONNECTED:  // 与服务端断开连接事件
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:  // 订阅成功事件
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:  // 取消订阅成功事件
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:  // 发布消息成功事件
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:   // 数据事件
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:   //错误事件
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

            }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;

    }

    return ESP_OK;
}

//mqtt注册的回调函数
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
   ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int)event_id);
   mqtt_event_handler_cb(event_data);
}

//mqtt开始函数，初始化mqtt客户端，并注册回调函数
void mqtt_app_start(void){
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://"iot_hostName,     //服务器地址
        .client_id = iot_clientID,           //客户端ID
        .username = iot_userName,             // 客户端用户名
        .password = iotda_pswd,  // 客户端登入密码
        .disable_auto_reconnect = false,              //自动重连功能
        .reconnect_timeout_ms = 1000,                 //断线重连事件间隔

    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);      //初始化mqtt客户端
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler,client);  //注册回调函数
    esp_mqtt_client_start(client);                 //启动mqtt客户端
}