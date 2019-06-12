
#include "step_motor_unit.h"

#define TAG 		"STEPPER"


//与l298n驱动板的连接(驱动板连接二相四线电机)
/*
	IN1	--	D21
	IN2	--	D19
	IN3	--	D18
	IN4	--	D5
*/
#define IN1    	21			//GPIO21
#define IN2   	19			//GPIO19
#define IN3    	18			//GPIO18
#define IN4    	5			//GPIO5
#define GPIO_OUTPUT_PIN_SEL (1ULL << IN1)|(1ULL << IN2)|(1ULL << IN3)|(1ULL << IN4)


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.001) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

//通过定时器的中断向步进电机发送脉冲
//中断函数里需要判断旋转的方向以及步进数
//初始化定时器
/**
 * 步进电机旋转任务，通过队列接收旋转结构体（包含旋转方向，步进数）
 * 开启定时器，旋转完成后，关闭定时器
 * 开启定时器即开始旋转，开启定时器的条件为接收到旋转指令或者电流激增（手拉电流）
 * 关闭定时器即停止旋转，关闭定时器条件为旋转结束或者发生堵转事件
 * 堵转判断条件为电流是否激增，从INA226实时读取电流
 */

//定时器累加
int timer_count = 0;
//相序累加
int phase_count = 0;

//旋转方向
int direction = 0;
//总步进个数
int step_count = 0;
//步进个数累加
int temp_step_count = 0;
//是否允许旋转
bool is_runnable = false;

//步进结构体
typedef struct
{
    int direction;
    int step_count;
}stepper_t;

//定时器队列，发送或接收步进结构体，控制定时器的开关(电机启动停止)
xQueueHandle timer_queue;

//定时器中断
void IRAM_ATTR timer_isr(void *arg)
{
    timer_count++;
    //因为中断的时间精度设置为1ms，即此中断每1ms被调用一次
    //每次脉冲延时2ms，也就是频率为500Hz，转速约150RPM
    if (timer_count == 2)
    { 
        timer_count = 0;
        //完成一个步进
        if (phase_count == 4)
        {
            //当前步进数
            temp_step_count++;
            //判断当前步进数是否已经达到步进总数
            if ( temp_step_count == step_count)
            {
                temp_step_count = 0;
                //向队列发送已完成步进的值
                //同时禁用旋转
                is_runnable = false;
            }
            phase_count = 0;
        }
        //确保步进电机可以马上停止
        if (is_runnable)
        {
            write_step_by_direction_and_phase(direction, phase_count);
            phase_count++;
        }
    }

    int timer_idx = (int) arg;
    //从定时器报告的中断中，获取中断状态以及计数器值
    //uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
    //为定时器（没有reload的情况下）清除中断以及更新警报时间
    //定时器Reload就是不累计时间，不Reload就是累计时间
    //比如设定时间为1s，则Reload的定时器每次到了1s之后就会重新开始计时，不Reload的定时器则会一直计时下去
    //Reload: 0s, 1s, 0s, 1s, 0s, 1s, ...
    //Without Reload: 0s, 1s, 2s, 3s, ...
    TIMERG0.int_clr_timers.t0 = 1;
    timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
    TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
    TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    //在警报触发之后，需要重新启用它，以便下次能够再触发
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

/**
 * 初始化定时器
 * @param timer_idx          定时器序号
 * @param auto_reload        定时器是否重载
 *                           重载不累计时间
 *                           不重载就一直累计时间
 * @param timer_interval_sec 时间精度（不是中断的精度）
 */
void timer_init_with_idx(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    //初始化定时器参数
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    //定时器的计数器将会从以下的数值开始计数，同时，如果auto_reload设置了，这个值会重新在警报上装载
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    //配置警报值以及警报的中断
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //timer_start(TIMER_GROUP_0, timer_idx);
}

//测试定时器的启动停止

//步进电机任务
void stepper_task(void *arg)
{
    ESP_LOGI(TAG, "步进电机任务开始");
}

//初始化GPIO
void step_gpio_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

//设置GPIO
void write_step(int a, int b, int c, int d) 
{
    gpio_set_level(IN1, a);
    gpio_set_level(IN2, b);
    gpio_set_level(IN3, c);
    gpio_set_level(IN4, d);
}

/**
 * 根据相序设置GPIO
 * @param direction 旋转的方向，1为正转，0为反转
 * @param phase     相序拍数
 */
void write_step_by_direction_and_phase(int direction, int phase)
{
    //正转相序
    if ( direction == 1)
    {
        write_step_by_phase_clockwise(phase);
    }
    //反转相序
    else
    {
        write_step_by_phase_counterclockwise(phase);
    }
}

//根据相序设置GPIO，正转
void write_step_by_phase_clockwise(int phase)
{
    switch(phase)
    {
        case 0:
            write_step(1, 0, 0, 0);
            break;
        case 1:
            write_step(0, 0, 1, 0);
            break;
        case 2:            
            write_step(0, 1, 0, 0);
            break;
        case 3:
            write_step(0, 0, 0, 1); 
            break;
        default:
            break;
    }
}

//根据相序设置GPIO，反转
void write_step_by_phase_counterclockwise(int phase)
{
    switch(phase)
    {
        case 0:
            write_step(0, 0, 0, 1);
            break;
        case 1:
            write_step(0, 1, 0, 0);
            break;
        case 2:            
            write_step(0, 0, 1, 0);
            break;
        case 3:
            write_step(1, 0, 0, 0); 
            break;
        default:
            break;
    }
}

//步进电机模块初始化
void stepper_init()
{
    //初始化GPIO
    step_gpio_init();
    //开辟队列空间
    timer_queue = xQueueCreate(10, sizeof(stepper_t));
    //初始化定时器
    timer_init_with_idx(TIMER_0, false, TIMER_INTERVAL0_SEC);
}

// /*
//  * A sample structure to pass events
//  * from the timer interrupt handler to the main program.
//  */
// typedef struct {
//     int type;  // the type of timer's event
//     int timer_group;
//     int timer_idx;
//     uint64_t timer_counter_value;
// } timer_event_t;

// xQueueHandle timer_queue;


/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
// //打印函数,打印计数器值以及时间（经过转换的时间）
// static void inline print_timer_counter(uint64_t counter_value)
// {
//     printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32), (uint32_t) (counter_value));
//     printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
// }

// //定时器中断函数
// void IRAM_ATTR timer_group0_isr(void *para)
// {
//     int timer_idx = (int) para;

//     //从定时器报告的中断中，获取中断状态以及计数器值
//     uint32_t intr_status = TIMERG0.int_st_timers.val;
//     TIMERG0.hw_timer[timer_idx].update = 1;
//     uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

//     //准备基础的事件数据，这些数据被发回给任务函数
//     timer_event_t evt;
//     evt.timer_group = 0;
//     evt.timer_idx = timer_idx;
//     evt.timer_counter_value = timer_counter_value;

//     time_count++;
//     if (time_count == 3)
//     { 
//         time_count = 0;
//         if (phase_count == 4)
//         {
//             phase_count = 0;
//         }
//         write_step_by_phase(phase_count);
//         phase_count++;
//     }

//     //为定时器（没有reload的情况下）清除中断以及更新警报时间
//     if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
//         evt.type = TEST_WITHOUT_RELOAD;
//         TIMERG0.int_clr_timers.t0 = 1;
//         timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
//         TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
//         TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
//     } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
//         evt.type = TEST_WITH_RELOAD;
//         TIMERG0.int_clr_timers.t1 = 1;
//     } else {
//         evt.type = -1; // not supported even type
//     }

//     //在警报触发之后，需要重新启用它，以便下次能够再触发
//     TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

//     //将事件数据发送给任务函数
//     xQueueSendFromISR(timer_queue, &evt, NULL);
// }

// //初始化定时器(隶属于定时器组0)
// static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec)
// {
//     //初始化定时器参数
//     timer_config_t config;
//     config.divider = TIMER_DIVIDER;
//     config.counter_dir = TIMER_COUNT_UP;
//     config.counter_en = TIMER_PAUSE;
//     config.alarm_en = TIMER_ALARM_EN;
//     config.intr_type = TIMER_INTR_LEVEL;
//     config.auto_reload = auto_reload;
//     timer_init(TIMER_GROUP_0, timer_idx, &config);

//     //定时器的计数器将会从以下的数值开始计数，同时，如果auto_reload设置了，这个值会重新在警报上装载
//     timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

//     //配置警报值以及警报的中断
//     timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, timer_idx);
//     timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

//     timer_start(TIMER_GROUP_0, timer_idx);
// }

// //定时器测试任务
// static void timer_example_evt_task(void *arg)
// {
//     int delay = 500;
//     while (1) {
//         timer_event_t evt;
//         //队列接收time_event结构体
//         xQueueReceive(timer_queue, &evt, portMAX_DELAY);
//         // writeStep(1, 0, 0, 0, delay);
//         // writeStep(0, 0, 1, 0, delay);
//         // writeStep(0, 1, 0, 0, delay);
//         // writeStep(0, 0, 0, 1, delay);
//         //ESP_LOGI(TAG, "time_count:%d", time_count);
//         // time_count++;
//         if ( time_count == 1000 )
//         {
//             time_count = 0;
//             ESP_LOGI(TAG, "1s时间到");
//         }
//         //print_timer_counter(evt.timer_counter_value);

//         // /* Print information that the timer reported an event */
//         // //打印定时器触发的数据
//         // if (evt.type == TEST_WITHOUT_RELOAD) {
//         //     printf("\n    Example timer without reload\n");
//         // } else if (evt.type == TEST_WITH_RELOAD) {
//         //     printf("\n    Example timer with auto reload\n");
//         // } else {
//         //     printf("\n    UNKNOWN EVENT TYPE\n");
//         // }
//         // printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

//         // /* Print the timer values passed by event */
//         // printf("------- EVENT TIME --------\n");
//         // print_timer_counter(evt.timer_counter_value);

//         // /* Print the timer values as visible by this task */
//         // printf("-------- TASK TIME --------\n");
//         // uint64_t task_counter_value;
//         // timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
//         // print_timer_counter(task_counter_value);
//     }
// }


//暂停以及启动定时器
void pause_timer_task(void *arg)
{
    while(1)
    {
        
        //暂停定时器
        timer_pause(TIMER_GROUP_0, TIMER_0);
        vTaskDelay( 1000 / portTICK_PERIOD_MS);

        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_enable_intr(TIMER_GROUP_0, TIMER_0);
        //启动定时器
        timer_start(TIMER_GROUP_0, TIMER_0);
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// //初始化定时器
// void init_timer()
// {
//     timer_queue = xQueueCreate(1000, sizeof(timer_event_t));
//     example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
//     //暂停计时器
//     //timer_pause(TIMER_GROUP_0, TIMER_0);
//     //example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
//     xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
//     //xTaskCreate(pause_timer_task, "pause_timer_task", 1024*4, NULL, 6, NULL);
//     //xTaskCreate(stepper_test_task, "stepper_test_task", 1024*4, NULL, 6, NULL);
// }

//定时器reload就是一次中断的时间精度
//定时器没有reload就是累计时间
//reload等同于是否累计时间

//定时器设置不需要reload（累计时间），如果时间累计达到步进电机的时间间隔，则reload
//中断里统计时间总和，如果时间总和为步进电机的时间间隔，则执行一次步进
//通过队列发送中断的执行情况
//
//定时器暂停以及重新启动

// void stepper_run(int direction, int count, int delay)
// {
//     //旋转次数缓冲区
//     int run_count = count;

//     //设置定时器中断精度
//     timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
//     timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL0_SEC * TIMER_SCALE);
//     timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//     //启动定时器
//     timer_start(TIMER_GROUP_0, TIMER_0);
    
//     timer_event_t evt;
//     while(run_count > 0)
//     {
//         //队列接收定时器中断通知
//         xQueueReceive(timer_queue, &evt, portMAX_DELAY);
//         if (direction == 1)
//         {
//             //顺时针旋转
//             writeStep(1, 0, 0, 0, delay);
//             writeStep(0, 0, 1, 0, delay);
//             writeStep(0, 1, 0, 0, delay);
//             writeStep(0, 0, 0, 1, delay);
//         }
//         else
//         {
//             //逆时针旋转
//         }
//         run_count--;
//     }
//     //暂停计时器
//     timer_pause(TIMER_GROUP_0, TIMER_0);
// }

// //测试任务
// void stepper_test_task(void *arg)
// {
// 	ESP_LOGI(TAG, "步进电机函数测试任务");
//     //STEPPER_STRUCT stepper;
//     //接收旋转通知，内容为一个结构体，包含旋转方向，旋转次数以及旋转速度
//     //获取结构体
//     //设置中断精度
//     //启用定时器
//     //接收定时器的通知
//     //暂停计时器
//     while(1)
//     {
//         ESP_LOGI(TAG, "步进电机旋转开始");
//         stepper_run(1, 200*50, 1);
//         ESP_LOGI(TAG, "步进电机旋转结束");
//         writeStep(0, 0, 0, 0, 1);
//         vTaskDelay( 5000 / portTICK_RATE_MS);
//     }
// 	vTaskDelete(NULL);
// }


//顺时针旋转
//参考正转表{0x05,0x01,0x09,0x08,0x0A,0x02,0x06,0x04}
		// writeStep(1, 0, 0, 0);
		// writeStep(1, 0, 1, 0);
		// writeStep(0, 0, 1, 0);
		// writeStep(0, 1, 1, 0);
		// writeStep(0, 1, 0, 0);
		// writeStep(0, 1, 0, 1);
		// writeStep(0, 0, 0, 1);
		// writeStep(1, 0, 0, 1);

//参考依据:反转表:{0x03, 0x09, 0x0c, 0x06}
		// writeStep(1, 0, 0, 1);
		// writeStep(0, 0, 0, 1);
		// writeStep(0, 1, 0, 1);
		// writeStep(0, 1, 0, 0);
		// writeStep(0, 1, 1, 0);
		// writeStep(0, 0, 1, 0);
		// writeStep(1, 0, 1, 0);
		// writeStep(1, 0, 0, 0);







