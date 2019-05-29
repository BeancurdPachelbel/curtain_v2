
#include "step_motor_unit.h"

#define TAG 		"STEPPER"

//高速模式定时器、高速模式
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (21)
//单独的channel
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

//低速定时器、低速模式
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (18)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

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

//PWM初始化
void stepper_pwm_init()
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 300,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    //等于设置了第一个timer，timer0
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    //修改速度模式为低速模式以及定时器类型为低速定时器，设置为timer1
    //也可以通过ledc_timer_config_t结构体创建另外一个timer对象
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    //单独配置每一个ledc_channel_config_t的channel, duty, gpio_num, speed_num, timer_sel属性
    //一下声明为数组
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
}


//测试任务
void stepper_test_task(void *arg)
{
	ESP_LOGI(TAG, "步进电机函数测试任务");
	ESP_LOGI(TAG, "低速运行");
	//启动
	stepper_run(10, 10);
	ESP_LOGI(TAG, "加速运行");
	while(1){
		//加速
		stepper_run(3, 10);
	}
	vTaskDelete(NULL);
}

// while (1) {
//         //duty值从0到LEDC_TEST_DUTY，以LED灯的角度看则是亮度为4000时
//         //printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
//         printf("1. Led灯用%dms把亮度值从当前状态渐变到%d\n",LEDC_TEST_FADE_TIME,LEDC_TEST_DUTY);
//         for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
//             //设置渐变的时间
//             ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
//                     ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
//             //开始渐变，异步
//             ledc_fade_start(ledc_channel[ch].speed_mode,
//                     ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
//         }
//         //主线程进行ms级延时，延时LEDC_TEST_FADE_TIME，此延时为主线程阻塞
//         vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//         //duty值从LEDC_TEST_DUTY到0
//         //printf("2. LEDC fade down to duty = 0\n");
//         printf("2. Led灯用%dms把亮度值从当前状态渐变到0\n",LEDC_TEST_FADE_TIME);
//         for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
//             ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
//                     ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
//             ledc_fade_start(ledc_channel[ch].speed_mode,
//                     ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
//         }

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







