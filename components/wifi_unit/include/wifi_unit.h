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


//检查是否已经保存SSID以及密码
bool is_saved_wifi();

//连接wifi
bool connect_wifi();


#endif