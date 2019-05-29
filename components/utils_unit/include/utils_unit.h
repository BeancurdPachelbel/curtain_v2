#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//#include "esp32/rom/ets_sys.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

/*********************全局变量区*********************/
//仅声明不定义

//防止重复定义
#ifndef _UTILS_H
#define _UTILS_H

//任务队列
EventGroupHandle_t task_event_group;

//任务标志位
int TASK_BIT;

//初始化常量
void utils_init();

//启用触摸传感器标志，如果模块重启，则必须接收到MQTT的消息之后才能启用
bool is_active;

#endif

/*********************全局变量区*********************/