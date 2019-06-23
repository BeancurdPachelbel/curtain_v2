#include "mqtt_unit.h"

#define		TAG 		"MQTT"

//mqtt server address
const char *mqtt_address = "mqtt://192.168.123.54";
//mqtt server port
const int mqtt_port = 1883;

char *curtain_command_topic = "/Bedroom/Curtain/Command";
char *curtain_status_topic = "/Bedroom/Curtain/Status";

char *topic_buffer;
char *payload_buffer;

esp_mqtt_client_handle_t client;

/**
 * @brief      The callback function of mqtt
 */
esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    //esp_mqtt_client_handle_t client = event->client;
    //int msg_id = 0;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            //subscribe the curtain command topic
            esp_mqtt_client_subscribe(event->client, curtain_command_topic, 1);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT SUBSCRIBED, msg_id=%d", event->msg_id);
            //mqtt_client, topic, data, len, qos, retain
            // msg_id = esp_mqtt_client_publish(client, "/ControlCenter/GPIO/16", "0", 0, 0, 0);
            // ESP_LOGI(TAG, "Published successfully, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
        	//Only if the qos is set to 1 or 2 that will receive the return receipt
            ESP_LOGI(TAG, "MQTT PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            //ESP_LOGI(TAG, "MQTT receive message");
            //printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            //printf("DATA=%.*s\r\n", event->data_len, event->data);
            asprintf(&topic_buffer, "%.*s", event->topic_len, event->topic);
            asprintf(&payload_buffer, "%.*s", event->data_len, event->data);
            //ESP_LOGI(TAG, "MQTT Received:topic = %s", topic_buffer);
            //ESP_LOGI(TAG, "MQTT Received:payload = %s", payload_buffer);
            //Perfom different operations according to message
            mqtt_check_message(topic_buffer, payload_buffer);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT ERROR");
            break;
        default:
            break;
    }
    return ESP_OK;
}

void mqtt_app_start()
{
	const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = mqtt_address,
        .port = mqtt_port,
        .event_handle = mqtt_event_handler,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

void mqtt_check_message(char *topic, char *payload)
{
	if ( strcmp(curtain_command_topic, topic) == 0)
	{
		// ESP_LOGI(TAG, "Curtain command topic, the payload is: %s", payload);
  //       float percentage = atof(payload) / 100;
  //       ESP_LOGI(TAG, "Received percentage:%f", percentage);
  //       int all_stepper_count = get_stepper_count_instance();
  //       int current_stepper_count = get_current_stepper_count();
  //       ESP_LOGI(TAG, "all_stepper_count:%d, current_stepper_count:%d", all_stepper_count, current_stepper_count);
  //       float current_percentage = ((float)current_stepper_count / (float)all_stepper_count);
  //       ESP_LOGI(TAG, "Current curtain closed percentage:%f", current_percentage);
  //       int currection_direction = get_current_direction();
  //       //将百分比换算成步进数
  //       int stepper_count = abs((int)((percentage - current_percentage) * all_stepper_count));
  //       ESP_LOGI(TAG, "will run stepper_count:%d", stepper_count);
  //       //根据方向以及步进数进行旋转
  //       //与当期的关闭百分比对比，如果大于当前的关闭比例，则往开启的方向旋转，小于当前开启比例则往关闭的方向旋转
  //       if ( percentage > current_percentage)
  //       {
  //           ESP_LOGI(TAG, "当前的currection_direction:%d", currection_direction);
  //           //发送电机运行任务
  //           send_stepper_run_task(0, stepper_count);
  //       }
  //       else if ( percentage < current_percentage)
  //       {
  //           // if (currection_direction == 1)
  //           // {
  //           //     currection_direction = 0;
  //           // }
  //           // else
  //           // {
  //           //     currection_direction = 1;
  //           // }
  //           ESP_LOGI(TAG, "当前取反的currection_direction:%d", currection_direction);
  //           send_stepper_run_task(1, stepper_count);    
  //       }

        ESP_LOGI(TAG, "Curtain command topic, the payload is: %s", payload);
        float percentage = atof(payload) / 100;
        ESP_LOGI(TAG, "Received percentage:%f", percentage);
        send_stepper_run_task(percentage);    
	}
}

void publish_curtain_status(int percentage)
{
	char *payload;
	asprintf(&payload, "%d", percentage);
	esp_mqtt_client_publish(client, curtain_status_topic, payload, 0, 0, 0);
}