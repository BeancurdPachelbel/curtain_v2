#include "wifi_unit.h"

#define     TAG     "WIFI"

//Wifi task group, ensure to block the code until the wifi connected successfully
static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;
int ESPTOUCH_DONE_BIT = BIT1;
//Determine wifi information is stored in flash
bool is_saved = false;
//Determine smartconfig is succeeded
bool is_smartconfig = false;

/**
 * @brief      Wifi callback function
 */
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) 
    {
        case SYSTEM_EVENT_STA_START:
            if (is_saved)
            {
                esp_wifi_connect();
            }
            else
            {
                xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
            }
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            //If system gets ip, release the CONNECTED_BIT
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

/**
 * @brief      Smartconfig callback function
 */
void smart_config_callback(smartconfig_status_t status, void *pdata)
{
    switch (status) 
    {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAG, "Smartconfig is waiting...");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAG, "Smartconfig is looking for channels...");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAG, "Smartconfig is retriving wifi ssid and password");
            break;
        case SC_STATUS_LINK:
            ESP_LOGI(TAG, "Smartconfig connected");
            wifi_config_t *wifi_config = pdata;
            ESP_LOGI(TAG, "WIFI SSID:%s", wifi_config->sta.ssid);
            ESP_LOGI(TAG, "WIFI PASSWORD:%s", wifi_config->sta.password);
            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config) );
            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAG, "Smartconfig over");
            if (pdata != NULL) 
            {
                is_smartconfig = true;
                uint8_t phone_ip[4] = { 0 };
                memcpy(phone_ip, (uint8_t* )pdata, 4);
                ESP_LOGI(TAG, "The phone ip: %d.%d.%d.%d", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
            }
            xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
    }
}

void smartconfig_task(void *parm)
{
    ESP_LOGI(TAG, "Smartconfig begins");
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    ESP_ERROR_CHECK( esp_smartconfig_start(smart_config_callback) );
    while (1) 
    {
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "Wifi connected");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "Smartconfig ends");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

bool connect_wifi()
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config;
    //Reading wifi information from flash
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &wifi_config));
    if (strcmp((const char *)(&wifi_config)->sta.ssid, "")==0 || strcmp((const char *)(&wifi_config)->sta.password, "")==0)
    {
        ESP_LOGW(TAG, "Detected the wifi information hasn’t stored in flash, starting the smartonfig task");
        ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_FLASH));
        ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK( esp_wifi_start() );
        is_saved = false;
        //Waiting for smartonfig finished
        xEventGroupWaitBits(wifi_event_group, ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if (is_smartconfig)
        {
            return true;
        }
        else
        {
            return false;
        }
    }else{
        is_saved = true;
        ESP_LOGI(TAG, "Detected the wifi information has stored in flash");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_LOGI(TAG, "Trying to connect wifi, SSID:[%s], password:[%s]", (&wifi_config)->sta.ssid, (&wifi_config)->sta.password);
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "Waiting for wifi connected...");
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "WIFI connected!");
        return true;
    }
}

