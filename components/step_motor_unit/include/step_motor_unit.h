
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
//#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "utils_unit.h"
#include "driver/ledc.h"
//防止重复定义
#ifndef _STEP_MOTOR_UNIT_H
#define _STEP_MOTOR_UNIT_H

//初始化GPIO
void step_gpio_init();

//测试任务
void stepper_test_task(void *arg);

//PWM初始化
void stepper_pwm_init();


#endif