#include "mqtt_unit.h"

#define		TAG 		"MQTT"

//mqtt server address
const char *mqtt_address = "mqtt://192.168.0.115";
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
		ESP_LOGI(TAG, "Curtain command topic, the payload is: %s", payload);
	}
}

void publish_curtain_status(int percentage)
{
	char *payload;
	asprintf(&payload, "%d", percentage);
	esp_mqtt_client_publish(client, curtain_status_topic, payload, 0, 0, 0);
}