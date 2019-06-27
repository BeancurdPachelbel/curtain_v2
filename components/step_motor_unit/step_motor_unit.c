
#include "step_motor_unit.h"

#define TAG 		"STEPPER"

#define START       0
#define END         1
#define MIDDLE      2

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
#define TIMER_INTERVAL0_SEC   (0.0001) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

extern TaskHandle_t read_current_handle;

extern void publish_curtain_status(int percentage);

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
//定时器累加参考值
int timer_count_max = 1;
//相序累加
int phase_count = 0;

//当前方向
int current_direction;
//当前步进数
int current_stepper_count;
//当前百分比
int current_percentage;
//mqtt的百分比
int mqtt_percentage;
//位置状态(当前开合百分比的状态值)
int position_status;

//旋转方向
int direction = 0;
//总步进个数
int step_count = 0;
//步进个数累加
int temp_step_count = 0;
//是否允许旋转
bool is_runnable = false;
//是否刚刚启动
bool is_just_running = false;

//停止旋转任务组
EventGroupHandle_t stepper_event_group;
int STOP_BIT = BIT0;

//确定行程任务组
EventGroupHandle_t stepper_travel_group;
int FINISH_BIT = BIT1;

//重置各个状态的值
void reset_status()
{
    timer_count = 0;
    phase_count = 0;
    //direction = 0;
    step_count = 0;
    temp_step_count = 0;
    is_runnable = false;
    is_just_running = false;
}

//步进结构体
typedef struct
{
    int direction;
    int step_count;
}stepper_t;

//发送或接收步进结构体，开启定时器，电机启动
xQueueHandle start_queue;
//发送或接受步进次数，关闭计时器，电机停止
xQueueHandle stop_queue;

//定时器中断
void IRAM_ATTR timer_isr(void *arg)
{
    timer_count++;
    //因为中断的时间精度设置为1ms，即此中断每1ms被调用一次
    //每次脉冲延时2ms，也就是频率为500Hz，转速约150RPM
    if (timer_count == timer_count_max)
    { 
        timer_count = 0;
        //完成一个步进
        if (phase_count == 4)
        {
            //判断当前步进数是否已经达到步进总数
            if ( temp_step_count == step_count)
            {
                //向队列发送已完成步进的值
                // xQueueSendFromISR(stop_queue, &temp_step_count, NULL);
                //同时禁用旋转
                is_runnable = false;
                write_step_by_direction_and_phase(direction, -1);
                xEventGroupSetBits(stepper_event_group, STOP_BIT);
            }
            temp_step_count++; 
            phase_count = 0;
        }
        //确保步进电机可以马上停止
        if (is_runnable)
        {
            write_step_by_direction_and_phase(direction, phase_count);
        }
        else
        {
            //没有按预期完成步进总数，说明出现异常
            if (temp_step_count < step_count)
            {
                write_step_by_direction_and_phase(direction, -1);
                //向队列发送已完成步进的值
                //xQueueSendFromISR(stop_queue, &temp_step_count, NULL);
                // xEventGroupSetBits(stepper_event_group, STOP_BIT);
            }
        }
        phase_count++;
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
        case -1:
            write_step(0, 0, 0, 0);
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
        case -1:
            write_step(0, 0, 0, 0);
        default:
            break;
    }
}

//获取当前方向
int get_current_direction()
{
    return current_direction;
}
//获取当前步进数
int get_current_stepper_count()
{
    return current_stepper_count;
}

//步进电机任务
void stepper_task(void *arg)
{
    ESP_LOGI(TAG, "步进电机任务开始");
    stepper_t stepper_instance;
    while(1)
    {
        //等待步进电机开始旋转
        if (xQueueReceive(start_queue, &stepper_instance, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "步进电机开始旋转，旋转的方向:%d, 步进次数:%d", stepper_instance.direction, stepper_instance.step_count);
            vTaskSuspend(read_current_handle);
            timer_count_max = 12;
            is_runnable = true;
            set_is_just_running(true);
            direction = stepper_instance.direction;
            step_count = stepper_instance.step_count;
            //启动定时器0
            timer_start(TIMER_GROUP_0, 0);
            //ESP_LOGI(TAG, "定时器启动");
            stepper_event_group = xEventGroupCreate();
            vTaskDelay( 50 / portTICK_RATE_MS);
            // timer_count_max = 1;
            // vTaskDelay( 2000 / portTICK_RATE_MS);
            vTaskResume(read_current_handle);
        }
        //等待步进电机旋转结束
        xEventGroupWaitBits(stepper_event_group, STOP_BIT, false, true, portMAX_DELAY);
        //ESP_LOGI(TAG, "步进电机旋转结束");
        //暂停定时器
        timer_pause(TIMER_GROUP_0, TIMER_0);
        //ESP_LOGI(TAG, "定时器暂停");
        current_direction = direction;
        current_stepper_count = temp_step_count;
        //设置当前百分比
        set_current_percentage();
        //判断队列接收的步进数是否等于预期的步进总数，如果不是，则说明出现异常
        if (step_count == temp_step_count)
        {
            ESP_LOGI(TAG, "步进电机按照预期完成步进，步进总数:%d", step_count);
        }
        else
        {
            //方向1为关闭窗帘，方向0为开启窗帘
            ESP_LOGW(TAG, "步进电机未按照预期完成步进，步进总数:%d, 已完成步进数:%d, 当前方向:%d", step_count, temp_step_count, direction);
            // ESP_LOGW(TAG, "is_runnable:%d", is_runnable);
            //如果确定行程的任务组不为空
            if ( stepper_travel_group != NULL )
            {
                //释放标志位
                xEventGroupSetBits(stepper_travel_group, FINISH_BIT);
                vTaskDelay(10/ portTICK_RATE_MS);
            }
        }
        //重置各个状态的值
        reset_status();
    }
    vTaskDelete(NULL);
}

//停止转动
void stop_running()
{
    //说明触发手拉电流
    ESP_LOGW(TAG, "遇阻停止转动");
    is_runnable = false;
    write_step(0, 0, 0, 0);
    //暂停定时器
    timer_pause(TIMER_GROUP_0, TIMER_0);
    xEventGroupSetBits(stepper_event_group, STOP_BIT);
}

//电机反转
void stepper_reverse()
{
    direction = !direction;
    stepper_t stepper_instance = {.direction = direction, .step_count = 5000};
    xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
}

//如果接收到0%或者100%的开合百分比，直接运行到堵转，然后设置百分比为0或者100
void set_current_percentage()
{
    //手拉启动电机，位置报告可能不精确，但是不影响实际运行行程
    //MQTT控制则可以保证精确性
    int all_stepper_count = get_stepper_count_instance();
    if (direction == 1)
    {
        //求出百分比，保留小数点后两位，再化为整数
        current_percentage = current_percentage - (int)((((float)current_stepper_count / (float)all_stepper_count)+0.005)*100);
    }
    else
    {
        current_percentage = current_percentage + (int)((((float)current_stepper_count / (float)all_stepper_count)+0.005)*100);
    }

    if (current_percentage > 100)
    {
        current_percentage = 100;
    }
    if (current_percentage < 0)
    {
        current_percentage = 0;
    }

    ESP_LOGI(TAG, "current percentage:%d%%", current_percentage);
    publish_curtain_status(current_percentage);
}

//发送电机运行任务
void send_stepper_run_task(int percentage)
{
    mqtt_percentage = percentage;
    ESP_LOGI(TAG, "current percentage:%d%%, expected percentage:%d%%", current_percentage, percentage);
    int all_stepper_count = get_stepper_count_instance();

    stepper_t stepper_instance;
    //运行到起点
    if (percentage == 0)
    {
        position_status = START;
        stepper_instance.direction = 1;
        stepper_instance.step_count = all_stepper_count * 10;
    }
    //运行到终点
    else if (percentage == 100)
    {
        position_status = END;
        stepper_instance.direction = 0;
        stepper_instance.step_count = all_stepper_count * 10;
    }
    //运行到指定位置
    else
    {
        //保留小数点后两位,统一浮点型精度
        float percentage_f = (float)percentage / 100;
        float current_percentage_f = (float)current_percentage / 100;
        position_status = MIDDLE;

        if ( current_percentage == percentage)
        {
            ESP_LOGI(TAG, "相同的百分比，不需要旋转");
            return;
        }
        //与当期的关闭百分比对比，如果大于当前的关闭比例，则往开启的方向旋转，小于当前开启比例则往关闭的方向旋转
        else if (current_percentage > percentage)
        {
            stepper_instance.direction = 1;
        }
        else if (current_percentage < percentage)
        {
            stepper_instance.direction = 0;
        }
        //将百分比换算成步进数
        stepper_instance.step_count = abs((int)((current_percentage_f - percentage_f) * all_stepper_count));
    }
    xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
}

//判断是否刚刚启动
bool get_is_just_running()
{
    return is_just_running;
}

//设置刚刚启动状态
void set_is_just_running(bool running)
{
    is_just_running = running;
}

//步进电机模块初始化
void stepper_init()
{
    //初始化GPIO
    step_gpio_init();
    //设置GPIO
    write_step(0, 0, 0, 0);
    //开辟队列空间
    start_queue = xQueueCreate(10, sizeof(stepper_t));
    stop_queue = xQueueCreate(10, sizeof(int));
    //初始化定时器
    timer_init_with_idx(TIMER_0, false, TIMER_INTERVAL0_SEC);
    //重置各个状态的值
    reset_status();
    
    //测试任务
    xTaskCreate(stepper_task, "stepper_task", 1024*4, NULL, 6, NULL);

    //第一次上电确定轨道行程
    curtain_track_travel_init();

    // //队列测试
    // stepper_t stepper_instance = {.direction = 1, .step_count = 1000};
    // //xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
    // while(1)
    // {
    //     stepper_instance.direction = !direction;   
    //     is_runnable = true;
    //     xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);        
    // }
}

//第一次上电确定轨道行程
void curtain_track_travel_init()
{
    vTaskDelay(1000 /portTICK_RATE_MS);
    int count = read_stepper_count();
    //读取Flash中保存的步进总数，如果为0说明尚未保存，需要确定行程
    if ( count == 0)
    {
        ESP_LOGI(TAG, "尚未保存步进总数，开始确定行程");

        //确定起点
        //电机开始旋转，方向为1，一旦发生堵转则认为是起点
        stepper_travel_group = xEventGroupCreate();
        stepper_t stepper_instance = {.direction = 1, .step_count = 5000};
        xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
        //等待确定行程任务组结束
        xEventGroupWaitBits(stepper_travel_group, FINISH_BIT, false, true, portMAX_DELAY);

        //确定终点
        //延时3s
        vTaskDelay(3000 / portTICK_RATE_MS);
        //电机开始旋转，方向为0，一旦发生堵转则认为是终点，并记录已发送的步进数
        stepper_travel_group = xEventGroupCreate();
        stepper_instance.direction = 0;
        xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
        //等待确定行程任务组结束
        xEventGroupWaitBits(stepper_travel_group, FINISH_BIT, false, true, portMAX_DELAY);
        //保存步进总数
        save_stepper_count(temp_step_count);
        
        //回到起点
        //延时3s
        vTaskDelay(3000 / portTICK_RATE_MS);
        //电机开始旋转，方向为1，向Flash读取步进总数，回到起点
        count = read_stepper_count();
        if ( count != 0)
        {
            stepper_instance.direction = 1;
            stepper_instance.step_count = count;
            xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
        }
    }
    else
    {
        ESP_LOGI(TAG, "起点到终点的步进总数为%d", count);
        current_percentage = 100;
        stepper_travel_group = xEventGroupCreate();
        stepper_t stepper_instance = {.direction = 1, .step_count = 5000};
        xQueueSend(start_queue, &stepper_instance, portMAX_DELAY);
        //等待确定行程任务组结束
        xEventGroupWaitBits(stepper_travel_group, FINISH_BIT, false, true, portMAX_DELAY);
        current_direction = 1;
        current_stepper_count = 0;
        mqtt_percentage = 0;
    }
}









