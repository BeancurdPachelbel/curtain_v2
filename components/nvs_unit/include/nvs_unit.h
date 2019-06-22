#ifndef __NVS_UNIT_H
#define __NVS_UNIT_H

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

/**
 * @brief       Save the whole curtain track travel stepper count
 */
void save_stepper_count(int count);


/**
 * @brief       Read the whole curtain track travel stepper count
 */
int read_stepper_count();


/**
 * @brief       get stepper count from flash 
 */
int get_stepper_count_instance();


#endif