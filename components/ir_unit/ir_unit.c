/* NEC remote infrared RMT example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
// #include "car_mp3_decode.h"

static const char* NEC_TAG = "NEC";

#define RMT_RX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */

#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  4     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         150                         /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */


/*
 * @brief Check whether duration is around target_us
 */
inline bool nec_check_in_range(int duration_ticks, int target_us, int margin_us)
{
    //ESP_LOGI(NEC_TAG, "nec_check_in_range - NEC_ITEM_DURATION = %d", NEC_ITEM_DURATION(duration_ticks));
    if(( NEC_ITEM_DURATION(duration_ticks) < (target_us + margin_us))
        && ( NEC_ITEM_DURATION(duration_ticks) > (target_us - margin_us))) {
        return true;
    } else {
        return false;
    }
}

/*
 * @brief Check whether this value represents an NEC header
 */
static bool nec_header_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
        && nec_check_in_range(item->duration0, NEC_HEADER_HIGH_US, NEC_BIT_MARGIN)
        && nec_check_in_range(item->duration1, NEC_HEADER_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents an NEC data bit 1
 */
static bool nec_bit_one_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
        && nec_check_in_range(item->duration0, NEC_BIT_ONE_HIGH_US, NEC_BIT_MARGIN)
        && nec_check_in_range(item->duration1, NEC_BIT_ONE_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}

/*
 * @brief Check whether this value represents an NEC data bit 0
 */
static bool nec_bit_zero_if(rmt_item32_t* item)
{
    if((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
        && nec_check_in_range(item->duration0, NEC_BIT_ZERO_HIGH_US, NEC_BIT_MARGIN)
        && nec_check_in_range(item->duration1, NEC_BIT_ZERO_LOW_US, NEC_BIT_MARGIN)) {
        return true;
    }
    return false;
}


/*
 * @brief Parse NEC 32 bit waveform to address and command.
 */
static int nec_parse_items(rmt_item32_t* item, int item_num, uint16_t* addr, uint16_t* data)
{
    int w_len = item_num;
    if(w_len < NEC_DATA_ITEM_NUM) {
        return -1;
    }
    int i = 0, j = 0;

    if(!nec_header_if(item++)) {
        ESP_LOGE(NEC_TAG, "nec_parse_items - HEADER KO");
        item--;
        for(i=0; i<item_num; i++)
            ESP_LOGI(NEC_TAG, "nec_parse_items - item[%d]: %dus@%d - %dus@%d",
                    i,
                    NEC_ITEM_DURATION(item[i].duration0),
                    NEC_ITEM_DURATION(item[i].level0),
                    NEC_ITEM_DURATION(item[i].duration1),
                    NEC_ITEM_DURATION(item[i].level1));

        return -1;
    }
    uint16_t addr_t = 0;
    for(j = 0; j < 16; j++) {
        if(nec_bit_one_if(item)) {
            //ESP_LOGI(NEC_TAG, "nec_parse_items - [%d] is LOGICAL 1", i);
            addr_t |= (1 << j);
        } else if(nec_bit_zero_if(item)) {
            //ESP_LOGI(NEC_TAG, "nec_parse_items - [%d] is LOGICAL 0", i);
            addr_t |= (0 << j);
        } else {
            //ESP_LOGE(NEC_TAG, "nec_parse_items - [%d] is LOGICAL 0", i);
            return -1;
        }
        item++;
        i++;
    }
    uint16_t data_t = 0;
    for(j = 0; j < 16; j++) {
        if(nec_bit_one_if(item)) {
            data_t |= (1 << j);
        } else if(nec_bit_zero_if(item)) {
            data_t |= (0 << j);
        } else {
            return -1;
        }
        item++;
        i++;
    }
    *addr = addr_t;
    *data = data_t;
    return i;
}


/*
 * @brief RMT receiver initialization
 */
static void nec_rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

/**
 * @brief RMT receiver demo, this task will print each received NEC data.
 *
 */
static void rmt_example_nec_rx_task()
{
    int channel = RMT_RX_CHANNEL;
    nec_rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    rmt_get_ringbuf_handle(channel, &rb);
    rmt_rx_start(channel, 1);
    while(rb) {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        //ESP_LOGI(NEC_TAG, "RMT RCV --- xRingbufferReceive: item=%p size=%d", item, rx_size);
        if(item) {
            uint16_t rmt_addr;
            uint16_t rmt_cmd;
            int offset = 0;
            while(1) {
                //parse data value from ringbuffer.
                int res = nec_parse_items(item + offset, rx_size / 4 - offset, &rmt_addr, &rmt_cmd);
                if(res > 0) {
                    offset += res + 1;
                    ESP_LOGI(NEC_TAG, "RMT RCV --- addr: 0x%04x cmd: 0x%04x", rmt_addr, rmt_cmd);
                    // car_mp3_decode_btn(rmt_addr, rmt_cmd);
                } else {
                    break;
                }
            }
            //after parsing the data, return spaces to ringbuffer.
            vRingbufferReturnItem(rb, (void*) item);
        } else {
            //ESP_LOGI(NEC_TAG, "RMT RCV --- Timeout");
            //break;
        }
    }
    vTaskDelete(NULL);
}

void ir_init()
{
	xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
}

// void app_main()
// {
//     xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
// }
