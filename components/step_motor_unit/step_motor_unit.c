
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


#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (1) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
//打印函数,打印计数器值以及时间（经过转换的时间）
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32), (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

//定时器中断函数
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    //从定时器报告的中断中，获取中断状态以及计数器值
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    TIMERG0.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;

    //准备基础的事件数据，这些数据被发回给任务函数
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    //为定时器（没有reload的情况下）清除中断以及更新警报时间
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        evt.type = TEST_WITHOUT_RELOAD;
        TIMERG0.int_clr_timers.t0 = 1;
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    } else if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        evt.type = TEST_WITH_RELOAD;
        TIMERG0.int_clr_timers.t1 = 1;
    } else {
        evt.type = -1; // not supported even type
    }

    //在警报触发之后，需要重新启用它，以便下次能够再触发
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    //将事件数据发送给任务函数
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
//初始化定时器(隶属于定时器组0)
static void example_tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    //初始化定时器参数
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    //定时器的计数器将会从以下的数值开始计数，同时，如果auto_reload设置了，这个值会重新在警报上装载
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    //配置警报值以及警报的中断
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task of this example program
 */
//定时器测试任务
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        //队列接收time_event结构体
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        //打印定时器触发的数据
        if (evt.type == TEST_WITHOUT_RELOAD) {
            printf("\n    Example timer without reload\n");
        } else if (evt.type == TEST_WITH_RELOAD) {
            printf("\n    Example timer with auto reload\n");
        } else {
            printf("\n    UNKNOWN EVENT TYPE\n");
        }
        printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);
    }
}

//暂停以及启动定时器
void pause_timer_task(void *arg)
{
    while(1)
    {
        //暂停定时器
        timer_pause(TIMER_GROUP_0, TIMER_0);
        vTaskDelay( 2000 );
    }
    vTaskDelete(NULL);
}

//初始化定时器
void init_timer()
{
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    //example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
}

//定时器reload就是一次中断的时间精度
//定时器没有reload就是累计时间
//reload等同于是否累计时间

//定时器设置不需要reload（累计时间），如果时间累计达到步进电机的时间间隔，则reload
//中断里统计时间总和，如果时间总和为步进电机的时间间隔，则执行一次步进
//通过队列发送中断的执行情况
//
//定时器暂停以及重新启动

//测试任务
void stepper_test_task(void *arg)
{
	ESP_LOGI(TAG, "步进电机函数测试任务");
    
	vTaskDelete(NULL);
}


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







