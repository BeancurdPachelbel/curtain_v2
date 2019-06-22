/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "driver/sdmmc_host.h"
#include "esp_event_loop.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "utils_unit.h"
#include "step_motor_unit.h"
#include "ina219_unit.h"
#include "wifi_unit.h"
#include "mqtt_unit.h"
#include "nvs_unit.h"

#define TAG     "MAIN"


void app_main(void)
{

    //nvs_flash_init();
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // //If wifi connected, try to connect mqtt server
    // if ( connect_wifi() )
    // {
    //     mqtt_app_start();
    // }
    // else
    // {
    //     ESP_LOGI(TAG, "Wifi is not connected, stop trying to connect mqtt server");
    // }


    // //初始化常量
    // utils_init();
    // 
    
    //初始化INA219
    ina219_init();
    
    //步进电机模块初始化
    stepper_init();

}
