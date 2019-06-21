#ifndef __WIFI_UNIT_H
#define __WIFI_UNIT_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_smartconfig.h"

// #include "utils_unit.h"

/**
 * @brief       Try to connect wifi
 * 				If the chip is first boot will switch to smartconfig
 * 				otherwise connects wifi directly by storaged wifi information
 *
 * @return     True if connected wifi, False otherwise
 */
bool connect_wifi();

/**
 * @brief      Smartconfig task, if wifi information is not stored in flash
 *              which means the chip boots first time, then deploy the smartconfig task
 * @param      parm  The parameter
 */
void smartconfig_task(void * parm);


#endif