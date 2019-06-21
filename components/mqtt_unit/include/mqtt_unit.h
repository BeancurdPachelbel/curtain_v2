#ifndef __MQTT_UNIT_
#define __MQTT_UNIT_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"


/**
 * @brief      The mqtt client starts to connect server 
 * 				and registers the callback function
 */
void mqtt_app_start();

/**
 * @brief      Peform different operations according to message
 *
 * @param      topic    The message's topic that client received
 * @param      payload  The message's payload that client received
 */
void mqtt_check_message(char *topic, char *payload);

/**
 * @brief      Publish the percentage of curtain opening and closing
 *
 * @param[in]  percentage  The percentage
 */
void publish_curtain_status(int percentage);


#endif