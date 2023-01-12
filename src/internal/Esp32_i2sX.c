// WARNING:  This file contains code that is more than likely already 
// exposed from the Esp32 Arduino API.  It will be removed once integration is complete.
//
// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------------------------------------

    High optimizated for speed using two parallel channels:
      - FillBuffer: one time pass for better performance
      - LUT tables support
      - other optimization for i2s handling   
    by @awawa-dev (https://github.com/awawa-dev)

-------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_ESP32) 

#include "sdkconfig.h" // this sets useful config symbols, like CONFIG_IDF_TARGET_ESP32C3

// ESP32 C3 & S3 I2S is not supported yet due to significant changes to interface
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"


#if ESP_IDF_VERSION_MAJOR>=4
#include "esp_intr_alloc.h"
#else
#include "esp_intr.h"
#endif

#include "rom/lldesc.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/i2s_struct.h"
#if defined(CONFIG_IDF_TARGET_ESP32)
/* included here for ESP-IDF v4.x compatibility */
#include "soc/dport_reg.h"
#endif
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/i2s.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#include "driver/dac.h"
#endif

#include "Esp32_i2s.h"
#include "esp32-hal.h"

esp_err_t i2sXSetClock(uint8_t bus_num, uint8_t div_num, uint8_t div_b, uint8_t div_a, uint8_t bck, uint8_t bits_per_sample);
esp_err_t i2sXSetSampleRate(uint8_t bus_num, uint32_t sample_rate, uint8_t bits_per_sample);

#define MATRIX_DETACH_OUT_SIG 0x100

#if ESP_IDF_VERSION_MAJOR<=4
#define I2S_BASE_CLK (160000000L)
#endif

#define I2S_DMA_BLOCK_COUNT_DEFAULT      16
// 24 bytes gives us enough time if we use single stage idle
// with the two stage idle we can use the minimum of 4 bytes
#define I2S_DMA_SILENCE_SIZE     4*1 
#define I2S_DMA_SILENCE_BLOCK_FRONT_COUNT 2 // two front
#define I2S_DMA_SILENCE_BLOCK_BACK_COUNT 5 // one back
#define I2S_DMA_QUEUE_COUNT 2

typedef struct 
{
    i2s_dev_t* bus;
    int8_t  ws;
    int8_t  bck;
    int8_t  out;
    int8_t  in;
    uint32_t rate;
    intr_handle_t isr_handle;

    uint8_t* silence_buf;
    size_t silence_len;

    lldesc_t* dma_items;
    size_t dma_count;
    uint32_t dma_buf_len :12;
    uint32_t unused      :20;
    volatile uint32_t is_sending_data;
} i2s_bus_t;

// is_sending_data values
#define I2s_Is_Idle 0
#define I2s_Is_Pending 1
#define I2s_Is_Sending 2

static uint8_t i2s_silence_buf[I2S_DMA_SILENCE_SIZE] = { 0 };

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
// (I2S_NUM_MAX == 2)
static i2s_bus_t I2SX[I2S_NUM_MAX] = 
{
    {&I2S0, -1, -1, -1, -1, 0, NULL, i2s_silence_buf, I2S_DMA_SILENCE_SIZE, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle},
    {&I2S1, -1, -1, -1, -1, 0, NULL, i2s_silence_buf, I2S_DMA_SILENCE_SIZE, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle}
};
#else
static i2s_bus_t I2SX[I2S_NUM_MAX] = 
{
    {&I2S0, -1, -1, -1, -1, 0, NULL, i2s_silence_buf, I2S_DMA_SILENCE_SIZE, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle}
};
#endif

void IRAM_ATTR i2sXDmaISR(void* arg);


bool i2sXInitDmaItems(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    if (I2SX[bus_num].dma_items != NULL) 
    {
        log_i("tx queue already initialized");
        // already set
        return true;
    }

    size_t dmaCount = I2SX[bus_num].dma_count;

    if (I2SX[bus_num].dma_items == NULL) 
    {
        I2SX[bus_num].dma_items = (lldesc_t*)heap_caps_malloc(dmaCount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (I2SX[bus_num].dma_items == NULL) 
        {
            log_e("MEM ERROR!");
            return false;
        }
    }

    int i, i2;
    lldesc_t* item = NULL;

    for(i=0; i< dmaCount; i++) 
    {
        i2 = (i+1) % dmaCount;
        item = &I2SX[bus_num].dma_items[i];
        item->eof = 0;
        item->owner = 1;
        item->sosf = 0;
        item->offset = 0;
        item->buf = I2SX[bus_num].silence_buf;
        item->size = I2SX[bus_num].silence_len;
        item->length = I2SX[bus_num].silence_len;
        item->qe.stqe_next = &I2SX[bus_num].dma_items[i2];   
    }

    return true;
}

bool i2sXDeinitDmaItems(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    if (!I2SX[bus_num].dma_items) 
    {
        return false; // nothing to deinit
    }

    heap_caps_free(I2SX[bus_num].dma_items);
    I2SX[bus_num].dma_items = NULL;

    return true;
}

// normal 4, 10, 63, 12, 16

esp_err_t i2sXSetClock(uint8_t bus_num, 
        uint8_t div_num, // 4     13
        uint8_t div_b,   // 10    20
        uint8_t div_a,   // 63    63
        uint8_t bck,     // 12    60 or 7
        uint8_t bits)    // 16    8
{
    if (bus_num >= I2S_NUM_MAX || div_a > 63 || div_b > 63 || bck > 63) 
    {
        return ESP_FAIL;
    }
    log_i("i2sSetClock bus %u, clkm_div_num %u, clk_div_a %u, clk_div_b %u, bck_div_num %u, bits_mod %u",
        bus_num,
        div_num,
        div_a,
        div_b,
        bck,
        bits);

    i2s_dev_t* i2s = I2SX[bus_num].bus;

    typeof(i2s->clkm_conf) clkm_conf;

    clkm_conf.val = 0;
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
    clkm_conf.clka_en = 0;
#else
    clkm_conf.clk_sel = 2;
#endif

    clkm_conf.clkm_div_a = div_a;
    clkm_conf.clkm_div_b = div_b;
    clkm_conf.clkm_div_num = div_num;
    i2s->clkm_conf.val = clkm_conf.val;

    typeof(i2s->sample_rate_conf) sample_rate_conf;
    sample_rate_conf.val = 0;
    sample_rate_conf.tx_bck_div_num = bck;
    sample_rate_conf.rx_bck_div_num = bck;
    sample_rate_conf.tx_bits_mod = bits;
    sample_rate_conf.rx_bits_mod = bits;
    i2s->sample_rate_conf.val = sample_rate_conf.val;

    return ESP_OK;
}

void i2sXSetPins(uint8_t bus_num, int8_t out, int8_t parallel, bool invert) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return;
    }

    if (out >= 0) 
    {
        pinMode(out, OUTPUT);

        uint32_t i2sSignal;
        
        #if !defined(CONFIG_IDF_TARGET_ESP32S2)
        if (bus_num == 1) 
        {
            if (parallel == -1)
            {
                i2sSignal = I2S1O_DATA_OUT23_IDX;
            }
            else
            {
                i2sSignal = I2S1O_DATA_OUT0_IDX + parallel;
            }
        }
        else if (parallel != -1)
        {
            i2sSignal = I2S0O_DATA_OUT8_IDX + parallel;
        }
        else
        #endif
        {
            if (parallel == -1)
            {
                i2sSignal = I2S0O_DATA_OUT23_IDX;
            }
            else
            {
                i2sSignal = I2S0O_DATA_OUT16_IDX + parallel;                
            }
        }

        gpio_matrix_out(out, i2sSignal, invert, false);
    } 
}

bool i2sXWriteDone(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    return (I2SX[bus_num].is_sending_data == I2s_Is_Idle);
}

void i2sXResetFifoDMA(i2s_dev_t* i2s)
{
    i2s->lc_conf.in_rst=1; i2s->lc_conf.out_rst=1; i2s->lc_conf.ahbm_rst=1; i2s->lc_conf.ahbm_fifo_rst=1;
    i2s->lc_conf.in_rst=0; i2s->lc_conf.out_rst=0; i2s->lc_conf.ahbm_rst=0; i2s->lc_conf.ahbm_fifo_rst=0;
    i2s->conf.tx_reset=1; i2s->conf.tx_fifo_reset=1; i2s->conf.rx_fifo_reset=1;
    i2s->conf.tx_reset=0; i2s->conf.tx_fifo_reset=0; i2s->conf.rx_fifo_reset=0;
}

void i2sXInit(uint8_t bus_num, 
        uint32_t bits_per_sample, 
        uint32_t sample_rate, 
        i2s_tx_chan_mod_t chan_mod, 
        i2s_tx_fifo_mod_t fifo_mod, 
        size_t dma_count, 
        size_t dma_len) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return;
    }

    assert(I2S_DMA_SILENCE_BLOCK_FRONT_COUNT >= 2);
        
    I2SX[bus_num].is_sending_data = I2s_Is_Sending;
    I2SX[bus_num].dma_count = dma_count +  I2S_DMA_SILENCE_BLOCK_FRONT_COUNT + I2S_DMA_SILENCE_BLOCK_BACK_COUNT;
    I2SX[bus_num].dma_buf_len = dma_len & 0xFFF;

    if (!i2sXInitDmaItems(bus_num)) 
    {
        return;
    }

    // eof to last data item, it will loop the queue to avoid the doubling    
    for(int i = 0; i < I2S_DMA_SILENCE_BLOCK_BACK_COUNT + 1; i++)
    {
        I2SX[bus_num].dma_items[I2SX[bus_num].dma_count - i - 1].eof = 1;        
    }
    // eof to loop first two segments
    I2SX[bus_num].dma_items[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT].eof = 1;

    #if !defined(CONFIG_IDF_TARGET_ESP32S2)
    if (bus_num) 
    {
        periph_module_enable(PERIPH_I2S1_MODULE);
    } 
    else 
    #endif
    {
        periph_module_enable(PERIPH_I2S0_MODULE);
    }

    esp_intr_disable(I2SX[bus_num].isr_handle);
    i2s_dev_t* i2s = I2SX[bus_num].bus;
    i2s->out_link.stop = 1;
    i2s->conf.tx_start = 0;
    i2s->int_ena.val = 0;
    i2s->int_clr.val = 0xFFFFFFFF;
    i2s->fifo_conf.dscr_en = 0;

    i2sXResetFifoDMA(i2s);

    i2s->conf2.val = 0;
    i2s->conf2.lcd_en = (bits_per_sample == 8);
    i2s->lc_conf.val = 0;
    i2s->lc_conf.out_eof_mode = 1;
    i2s->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
    i2s->fifo_conf.val = 0;
    i2s->fifo_conf.tx_fifo_mod = fifo_mod;
    i2s->conf_chan.val = 0;
    i2s->conf_chan.tx_chan_mod = chan_mod; //  0-two channel;1-right;2-left;3-righ;4-left
    i2s->conf.val = 0;    
    i2s->timing.val = 0;
    i2s->fifo_conf.tx_fifo_mod_force_en = 1;
    i2s->conf1.tx_pcm_bypass = 1; 

    i2sXSetSampleRate(bus_num, sample_rate, bits_per_sample);

    i2s->fifo_conf.val = 0;
    i2s->fifo_conf.tx_fifo_mod_force_en = 1; // HN
    i2s->fifo_conf.tx_fifo_mod = 1;
    i2s->fifo_conf.tx_data_num = 32;
   
    //  enable intr in cpu
    int i2sIntSource = ETS_I2S0_INTR_SOURCE;

    #if !defined(CONFIG_IDF_TARGET_ESP32S2)
        i2sIntSource = (bus_num == 1) ? ETS_I2S1_INTR_SOURCE : ETS_I2S0_INTR_SOURCE;
    #endif

    esp_intr_alloc(i2sIntSource, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, 
                    &i2sXDmaISR, &I2SX[bus_num], &I2SX[bus_num].isr_handle);

    //  enable send intr
    i2s->int_ena.out_eof = 1;
    i2s->int_ena.out_dscr_err = 1;

    // reset FIFO/DMA
    i2sXResetFifoDMA(i2s);

    // Enable and configure DMA
    i2s->fifo_conf.dscr_en = 1;// enable dma
    i2s->out_link.start = 0;
    i2s->out_link.addr = (uint32_t)(&I2SX[bus_num].dma_items[0]); // loads dma_struct to dma
    i2s->out_link.start = 1; // starts dma
    i2s->conf.tx_start = 1;// Start I2s module

    esp_intr_enable(I2SX[bus_num].isr_handle);
}

void i2sXDeinit(uint8_t bus_num) 
{
    i2sXDeinitDmaItems(bus_num);
}

esp_err_t i2sXSetSampleRate(uint8_t bus_num, uint32_t rate, uint8_t bits) 
{
    // parallel modes need higher frequency on esp32
    int clockDivider =  (bus_num == 0) ? 4 : 2;

    if (bus_num >= I2S_NUM_MAX) 
    {
        return ESP_FAIL;
    }


    // 160,000,000L / (100,000 * 384)
    double clkmdiv = (double)I2S_BASE_CLK / ((rate * 384) + 1);
    if (clkmdiv > 256) 
    {
        log_e("rate is too low");
        return ESP_FAIL;
    } 
    else if (clkmdiv < 2) 
    {
        log_e("rate is too fast");
        return ESP_FAIL;
    }

    I2SX[bus_num].rate = rate;

    // calc integer and franctional for more precise timing
    // 
    uint8_t clkmInteger = clkmdiv;
    uint8_t clkmFraction = (clkmdiv - clkmInteger) * 63.0;
    
    // adjust bck clock divider here
    uint8_t bck = (bits == 8) ? 24 / clockDivider : 12;

    i2sXSetClock(bus_num, clkmInteger, clkmFraction, 63, bck, bits);

    return ESP_OK;
}

void IRAM_ATTR i2sXDmaISR(void* arg)
{
    i2s_bus_t* i2s = (i2s_bus_t*)(arg);

    if (i2s->bus->int_st.out_eof) 
    {
        if (i2s->is_sending_data == I2s_Is_Pending)
        {
            i2s->is_sending_data = I2s_Is_Idle;
        }
        else if (i2s->is_sending_data == I2s_Is_Sending)
        {
            // loop the silent items
            lldesc_t* itemSilence = &i2s->dma_items[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT - 1];
            itemSilence->qe.stqe_next = &i2s->dma_items[0];

            i2s->is_sending_data = I2s_Is_Pending;
        }
    }

    i2s->bus->int_clr.val = i2s->bus->int_st.val;
}

size_t i2sXWrite(uint8_t bus_num, uint8_t* data, size_t len, bool copy, bool free_when_sent) 
{
    if (bus_num >= I2S_NUM_MAX || I2SX[bus_num].dma_items == NULL) 
    {
        return 0;
    }
    size_t blockSize = len;

    lldesc_t* item = &I2SX[bus_num].dma_items[0]; 
    size_t dataLeft = len;
    uint8_t* pos = data;

    // skip front two silent items
    item += I2S_DMA_SILENCE_BLOCK_FRONT_COUNT;

    while (dataLeft) 
    {
        blockSize = dataLeft;
        if (blockSize > I2S_DMA_MAX_DATA_LEN) 
        {
            blockSize = I2S_DMA_MAX_DATA_LEN;
        }
        dataLeft -= blockSize;

        // data is constant. no need to copy
        item->buf = pos;
        item->size = blockSize;
        item->length = blockSize;

        item++;

        pos += blockSize;
    }

    I2SX[bus_num].is_sending_data = I2s_Is_Sending;    
    I2SX[bus_num].dma_items[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT-1].qe.stqe_next = &I2SX[bus_num].dma_items[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT];    
    
    return len;
}

#endif // !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#endif // defined(ARDUINO_ARCH_ESP32) 

