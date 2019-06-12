
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_types.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "utils_unit.h"
//防止重复定义
#ifndef _STEP_MOTOR_UNIT_H
#define _STEP_MOTOR_UNIT_H

struct stepper_struct
{
	int direction;
	int step_count;
	int speed;
};

typedef struct stepper_struct STEPPER_STRUCT;

//初始化GPIO
void step_gpio_init();

//测试任务
void stepper_test_task(void *arg);

//初始化定时器
void init_timer();

//初始化GPIO
void step_gpio_init();


void stepper_run(int direction, int count, int delay);

/**
 * 根据相序设置GPIO
 * @param direction 旋转的方向，1为正转，0为反转
 * @param phase     相序拍数
 */
void write_step_by_direction_and_phase(int direction, int phase);

/**
 * 根据相序设置GPIO
 * @param direction 旋转的方向，1为正转，0为反转
 * @param phase     相序拍数
 */
void write_step_by_direction_and_phase(int direction, int phase);

//根据相序设置GPIO，正转
void write_step_by_phase_clockwise(int phase);

//根据相序设置GPIO，反转
void write_step_by_phase_counterclockwise(int phase);

//步进电机模块初始化
void stepper_init();


#endif