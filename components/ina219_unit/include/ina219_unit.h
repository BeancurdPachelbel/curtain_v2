#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "step_motor_unit.h"


#ifndef __INA219_UNIT_H
#define __INA219_UNIT_H

TaskHandle_t read_current_handle;

//I2C总线初始化
void i2c_init();

//设置INA219的测量参数
void ina219_setCalibration_32V_2A();

//初始化INA219
void ina219_init();

//INA219写寄存器
esp_err_t ina219_writeRegister(uint8_t reg, uint16_t data);

//INA219读寄存器
esp_err_t ina219_readRegister(uint8_t reg, uint16_t *data);

//读取电流(二进制值)
int16_t ina219_getCurrent_raw();

//读取电流(浮点值)
float ina219_getCurrent_mA();

//测试读取电流值任务
void read_current_task(void *arg);

#endif