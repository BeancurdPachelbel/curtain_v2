#include "wifi_unit.h"

#define     TAG     "WIFI"

//WIFI任务组，WIFI连接标志位，等待WIFI连接成功之后再进行下一个人物(任务同步)
static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;
int ESPTOUCH_DONE_BIT = BIT1;
bool is_saved = false;

void smartconfig_task(void * parm);

//WIFI连接回调函数
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            if (is_saved){
                // is_first_boot = false;
                esp_wifi_connect();
            }else{
                // is_first_boot = true;
                xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
            }
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            //直到WIFI获取IP地址释放标志位
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

//Smartconfig回调函数
void smart_config_callback(smartconfig_status_t status, void *pdata)
{
    switch (status) {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAG, "Smartconfig等待...");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAG, "Smartconfig正在查找信道...");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAG, "Smartconfig正在获取WIFI的账号或者密码");
            break;
        case SC_STATUS_LINK:
            ESP_LOGI(TAG, "Smartconfig连接成功");
            wifi_config_t *wifi_config = pdata;
            ESP_LOGI(TAG, "WIFI的SSID:%s", wifi_config->sta.ssid);
            ESP_LOGI(TAG, "WIFI的PASSWORD:%s", wifi_config->sta.password);
            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config) );
            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAG, "Smartconfig连接结束");
            if (pdata != NULL) {
                uint8_t phone_ip[4] = { 0 };
                memcpy(phone_ip, (uint8_t* )pdata, 4);
                ESP_LOGI(TAG, "手机IP: %d.%d.%d.%d", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
            }
            xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
    }
}

void smartconfig_task(void * parm)
{
    ESP_LOGI(TAG, "Smartconfig开始");
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    ESP_ERROR_CHECK( esp_smartconfig_start(smart_config_callback) );
    while (1) {
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "已经连接WIFI");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "Smartconfig结束");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

//连接WIFI
bool connect_wifi()
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config;
    //从本地读取SSID以及Password
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config));
    if (strcmp((const char *)(&wifi_config)->sta.ssid, "")==0 || strcmp((const char *)(&wifi_config)->sta.password, "")==0)
    {
        ESP_LOGW(TAG, "没有检测到保存的SSID以及Password,进行Smartconfig");
        ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_FLASH));
        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK( esp_wifi_start() );
        is_saved = false;
        //等待WIFI连接成功
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, true, false, portMAX_DELAY); 
        ESP_LOGI(TAG, "WIFI连接成功!");
        return false;
    }else{
        is_saved = true;
        ESP_LOGI(TAG, "检测已经保存的SSID以及Password");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_LOGI(TAG, "开始连接WIFI, SSID:[%s], password:[%s]", (&wifi_config)->sta.ssid, (&wifi_config)->sta.password);
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "等待WIFI连接成功...");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "WIFI连接成功!");
        return true;
    }
}

