#include "ir_unit.h"

#define TIMER_DIVIDER         16  	//  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.00001) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define TAG	"IR"

//可尝试增加红外学习模式

//通过定时器中断实现精确计时
int timer1_count = 0;
int timer1_count_max = 0;
//定时器队列
xQueueHandle timer_queue;

//定时器中断
void IRAM_ATTR timer1_isr(void *arg)
{
    timer1_count++;
    //定时器精度为10us
    if (timer1_count == timer1_count_max)
    { 
        timer1_count = 0;
        xQueueSendFromISR(timer_queue, &timer1_count, NULL);
    }

    int timer_idx = (int) arg;
    //从定时器报告的中断中，获取中断状态以及计数器值
    //uint32_t intr_status = TIMERG1.int_st_timers.val;
    TIMERG1.hw_timer[timer_idx].update = 1;
    uint64_t timer_counter_value = ((uint64_t) TIMERG1.hw_timer[timer_idx].cnt_high) << 32 | TIMERG1.hw_timer[timer_idx].cnt_low;
    //为定时器（没有reload的情况下）清除中断以及更新警报时间
    //定时器Reload就是不累计时间，不Reload就是累计时间
    //比如设定时间为1s，则Reload的定时器每次到了1s之后就会重新开始计时，不Reload的定时器则会一直计时下去
    //Reload: 0s, 1s, 0s, 1s, 0s, 1s, ...
    //Without Reload: 0s, 1s, 2s, 3s, ...
    TIMERG1.int_clr_timers.t0 = 1;
    timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
    TIMERG1.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_counter_value >> 32);
    TIMERG1.hw_timer[timer_idx].alarm_low = (uint32_t) timer_counter_value;
    //在警报触发之后，需要重新启用它，以便下次能够再触发
    TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
}

/**
 * 初始化定时器
 * @param timer_idx          定时器序号
 * @param auto_reload        定时器是否重载
 *                           重载不累计时间
 *                           不重载就一直累计时间
 * @param timer_interval_sec 时间精度（不是中断的精度）
 */
void timer_init_with_timer1(int timer_idx, bool auto_reload, double timer_interval_sec)
{
    //初始化定时器参数
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_1, timer_idx, &config);

    //定时器的计数器将会从以下的数值开始计数，同时，如果auto_reload设置了，这个值会重新在警报上装载
    timer_set_counter_value(TIMER_GROUP_1, timer_idx, 0x00000000ULL);

    //配置警报值以及警报的中断
    timer_set_alarm_value(TIMER_GROUP_1, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_1, timer_idx);
    timer_isr_register(TIMER_GROUP_1, timer_idx, timer1_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //timer_start(TIMER_GROUP_0, timer_idx);
}

//初始化红外
void ir_init()
{
	timer1_count_max = 100000;
	timer_queue = xQueueCreate(10, sizeof(int));
	//初始化定时器1
	timer_init_with_timer1(TIMER_0, false, TIMER_INTERVAL0_SEC);
	int i = 0;
	//启动定时器1
	timer_start(TIMER_GROUP_1, TIMER_0);
	while(1)
	{
		if(xQueueReceive(timer_queue, &i, portMAX_DELAY))
		{
			ESP_LOGI(TAG, "%ds时间到", (int)(timer1_count_max*TIMER_INTERVAL0_SEC));
		}
	}
}

/**
 * 红外编码解释
 * 引导码+四个字节
 * 1.用户码
 * 2.用户码反码
 * 3.键码
 * 4.键码反码
 */

/**
 * 红外接收解码逻辑
 * 1.低电平的时间为9ms
 * 2.高电平的时间为4.5ms
 * 3.循环接收4个字节
 * 	3.1循环接收8个位
 * 	3.2低电平的时间为560us，为0
 * 	3.3高电平的时间为560us
 * 		3.3.1高电平的时间为1680us，为1
 * 	3.4接收一位，回到步骤3.1
 * 4.4个字节接收完毕
 */

