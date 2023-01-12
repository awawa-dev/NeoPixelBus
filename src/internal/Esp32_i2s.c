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

esp_err_t i2sSetClock(uint8_t bus_num, uint8_t div_num, uint8_t div_b, uint8_t div_a, uint8_t bck, uint8_t bits_per_sample);
esp_err_t i2sSetSampleRate(uint8_t bus_num, uint32_t sample_rate, uint8_t bits_per_sample);

#define MATRIX_DETACH_OUT_SIG 0x100

#if ESP_IDF_VERSION_MAJOR<=4
#define I2S_BASE_CLK (160000000L)
#endif

#define I2S_DMA_BLOCK_COUNT_DEFAULT      16
// 24 bytes gives us enough time if we use single stage idle
// with the two stage idle we can use the minimum of 4 bytes
#define I2S_DMA_SILENCE_SIZE     64 // 4 byte increments 
#define I2S_DMA_SILENCE_BLOCK_COUNT  3 // two front, one back
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

#define I2S_DMA_STATIC_SILENCE_SIZE     4 // 4 byte increments 
static uint8_t* s_i2s_silence_buf = NULL;

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
// (I2S_NUM_MAX == 2)
static i2s_bus_t I2S[I2S_NUM_MAX] = 
{
    {&I2S0, -1, -1, -1, -1, 0, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle},
    {&I2S1, -1, -1, -1, -1, 0, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle}
};
#else
static i2s_bus_t I2S[I2S_NUM_MAX] = 
{
    {&I2S0, -1, -1, -1, -1, 0, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, 0, 0, I2s_Is_Idle}
};
#endif

void IRAM_ATTR i2sDmaISR(void* arg);


bool i2sInitDmaItems(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    size_t dmaCount = I2S[bus_num].dma_count;

    if (s_i2s_silence_buf == NULL)
    {
        s_i2s_silence_buf = heap_caps_malloc(I2S_DMA_STATIC_SILENCE_SIZE, MALLOC_CAP_DMA);
        memset(s_i2s_silence_buf, 0x00, I2S_DMA_STATIC_SILENCE_SIZE);
    }

    if (I2S[bus_num].dma_items == NULL) 
    {
        I2S[bus_num].dma_items = (lldesc_t*)heap_caps_malloc(dmaCount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (I2S[bus_num].dma_items == NULL) 
        {
            log_e("MEM ERROR!");
            return false;
        }
    }

    lldesc_t* item = NULL;

    for (size_t i = 0; i < dmaCount; i++) 
    {
        size_t i2 = (i+1) % dmaCount;
        item = &I2S[bus_num].dma_items[i];
        item->eof = 0;
        item->owner = 1;
        item->sosf = 0;
        item->offset = 0;
        item->buf = s_i2s_silence_buf;
        item->size = I2S_DMA_SILENCE_SIZE;
        item->length = I2S_DMA_SILENCE_SIZE;
        item->qe.stqe_next = &I2S[bus_num].dma_items[i2];   
    }

    I2S[bus_num].dma_items[dmaCount - 2].eof = 1;
    I2S[bus_num].dma_items[dmaCount - 1].eof = 1;

    return true;
}

bool i2sDeinitDmaItems(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    heap_caps_free(I2S[bus_num].dma_items);
    I2S[bus_num].dma_items = NULL;

    return true;
}

// normal 4, 10, 63, 12, 16

esp_err_t i2sSetClock(uint8_t bus_num, 
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
/*
    log_i("i2sSetClock bus %u, clkm_div_num %u, clk_div_a %u, clk_div_b %u, bck_div_num %u, bits_mod %u",
        bus_num,
        div_num,
        div_a,
        div_b,
        bck,
        bits);
*/

    i2s_dev_t* i2s = I2S[bus_num].bus;

    typeof(i2s->clkm_conf) clkm_conf;

    clkm_conf.val = 0;
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
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

void i2sSetPins(uint8_t bus_num, int8_t out, int8_t parallel, bool invert) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return;
    }
    /* cant really track this with parrallel
    int8_t outOld = I2S[bus_num].out;

    I2S[bus_num].out = out;

    // disable old pin
    if (outOld >= 0)
    {
        gpio_matrix_out(outOld, MATRIX_DETACH_OUT_SIG, false, false);
        pinMode(outOld, INPUT);
    }
    */
    if (out >= 0) 
    {
        pinMode(out, OUTPUT);

        uint32_t i2sSignal;
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
//            (I2S_NUM_MAX == 2)
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

bool i2sWriteDone(uint8_t bus_num) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return false;
    }

    return (I2S[bus_num].is_sending_data == I2s_Is_Idle);
}

void i2sInit(uint8_t bus_num, 
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

    I2S[bus_num].dma_count = dma_count + I2S_DMA_SILENCE_BLOCK_COUNT; 
    I2S[bus_num].dma_buf_len = dma_len & 0xFFF;

    if (!i2sInitDmaItems(bus_num)) 
    {
        return;
    }

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
// (I2S_NUM_MAX == 2)
    if (bus_num) 
    {
        periph_module_enable(PERIPH_I2S1_MODULE);
    } 
    else 
#endif
    {
        periph_module_enable(PERIPH_I2S0_MODULE);
    }

    esp_intr_disable(I2S[bus_num].isr_handle);
    i2s_dev_t* i2s = I2S[bus_num].bus;
    i2s->out_link.stop = 1;
    i2s->conf.tx_start = 0;
    i2s->int_ena.val = 0;
    i2s->int_clr.val = 0xFFFFFFFF;
    i2s->fifo_conf.dscr_en = 0;

    // reset i2s
    i2s->conf.tx_reset = 1;
    i2s->conf.tx_reset = 0;
    i2s->conf.rx_reset = 1;
    i2s->conf.rx_reset = 0;

    // reset dma
    i2s->lc_conf.in_rst = 1;
    i2s->lc_conf.in_rst = 0;
    i2s->lc_conf.out_rst = 1;
    i2s->lc_conf.out_rst = 0;

    // reset fifo
    i2s->conf.rx_fifo_reset = 1;
    i2s->conf.rx_fifo_reset = 0;
    i2s->conf.tx_fifo_reset = 1;
    i2s->conf.tx_fifo_reset = 0;


    // set parallel (LCD) mode
    {
        typeof(i2s->conf2) conf2;
        conf2.val = 0;
        conf2.lcd_en = (bits_per_sample == 8 || bits_per_sample == 16);
        conf2.lcd_tx_wrx2_en = 0;//(bits_per_sample == 8 || bits_per_sample == 16);
        i2s->conf2.val = conf2.val;
    }

    // Enable and configure DMA
    {
        typeof(i2s->lc_conf) lc_conf;
        lc_conf.val = 0;
        lc_conf.out_eof_mode = 1;
        i2s->lc_conf.val = lc_conf.val;
    }

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    i2s->pdm_conf.pcm2pdm_conv_en = 0;
    i2s->pdm_conf.pdm2pcm_conv_en = 0;
#endif
    // SET_PERI_REG_BITS(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL, 0x1, RTC_CNTL_SOC_CLK_SEL_S);

    {
        typeof(i2s->fifo_conf) fifo_conf;
        fifo_conf.val = 0;
        fifo_conf.tx_fifo_mod = fifo_mod; //  0-right&left channel;1-one channel
        fifo_conf.rx_fifo_mod = fifo_mod; //  0-right&left channel;1-one channel
        i2s->fifo_conf.val = fifo_conf.val;
    }

    {
        typeof(i2s->conf1) conf1;
        conf1.val = 0;
        conf1.tx_stop_en = 0;
        conf1.tx_pcm_bypass = 1;
        i2s->conf1.val = conf1.val;
    }

    {
        typeof(i2s->conf_chan) conf_chan;
        conf_chan.val = 0;
        conf_chan.tx_chan_mod = chan_mod; //  0-two channel;1-right;2-left;3-righ;4-left
        conf_chan.rx_chan_mod = chan_mod; //  0-two channel;1-right;2-left;3-righ;4-left
        i2s->conf_chan.val = conf_chan.val;
    }

    {
        typeof(i2s->conf) conf;
        conf.val = 0;
        conf.tx_msb_shift = 0; // (bits_per_sample != 8);// 0:DAC/PCM, 1:I2S
        conf.tx_right_first = 0; // (bits_per_sample == 8);
        i2s->conf.val = conf.val;
    }

    i2s->timing.val = 0;

    i2s->fifo_conf.tx_fifo_mod_force_en = 1;

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    i2s->pdm_conf.rx_pdm_en = 0;
    i2s->pdm_conf.tx_pdm_en = 0;
#endif

    
    
    i2sSetSampleRate(bus_num, sample_rate, bits_per_sample);

    if (bits_per_sample == 8 || bits_per_sample == 16)
    {
        i2s->fifo_conf.val = 0;
        i2s->fifo_conf.rx_fifo_mod_force_en = 1; 
        i2s->fifo_conf.tx_fifo_mod_force_en = 1; // HN
        i2s->fifo_conf.rx_fifo_mod = 1; // HN
        i2s->fifo_conf.tx_fifo_mod = 1;
        i2s->fifo_conf.rx_data_num = 32; //Thresholds. 
        i2s->fifo_conf.tx_data_num = 32;
    }

    /* */
    //Reset FIFO/DMA -> needed? Doesn't dma_reset/fifo_reset do this?
    i2s->lc_conf.in_rst=1; i2s->lc_conf.out_rst=1; i2s->lc_conf.ahbm_rst=1; i2s->lc_conf.ahbm_fifo_rst=1;
    i2s->lc_conf.in_rst=0; i2s->lc_conf.out_rst=0; i2s->lc_conf.ahbm_rst=0; i2s->lc_conf.ahbm_fifo_rst=0;
    i2s->conf.tx_reset=1; i2s->conf.tx_fifo_reset=1; i2s->conf.rx_fifo_reset=1;
    i2s->conf.tx_reset=0; i2s->conf.tx_fifo_reset=0; i2s->conf.rx_fifo_reset=0;
    /* */

    //  enable intr in cpu // 
    int i2sIntSource;

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
//    (I2S_NUM_MAX == 2)
    if (bus_num == 1) {
        i2sIntSource = ETS_I2S1_INTR_SOURCE;
    }
    else
#endif
    {
        i2sIntSource = ETS_I2S0_INTR_SOURCE;
    }


    esp_intr_alloc(i2sIntSource, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, &i2sDmaISR, &I2S[bus_num], &I2S[bus_num].isr_handle);
    //  enable send intr
    i2s->int_ena.out_eof = 1;
    i2s->int_ena.out_dscr_err = 1;

/*  ??? */
    // Enable and configure DMA
    {
        typeof(i2s->lc_conf) lc_conf;
        lc_conf.val = 0;
        lc_conf.out_data_burst_en = 1;
        lc_conf.indscr_burst_en = 1;
        i2s->lc_conf.val = lc_conf.val;
    }
/* */
    i2s->fifo_conf.dscr_en = 1;// enable dma
    i2s->out_link.start = 0;
    i2s->out_link.addr = (uint32_t)(&I2S[bus_num].dma_items[0]); // loads dma_struct to dma
    i2s->out_link.start = 1; // starts dma
    i2s->conf.tx_start = 1;// Start I2s module

    esp_intr_enable(I2S[bus_num].isr_handle);
}

void i2sDeinit(uint8_t bus_num) 
{
    i2sDeinitDmaItems(bus_num);
}

esp_err_t i2sSetSampleRate(uint8_t bus_num, uint32_t rate, uint8_t bits) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return ESP_FAIL;
    }

    if (I2S[bus_num].rate == rate) 
    {
        return ESP_OK;
    }

    //               160,000,000L / (100,000 * 384)
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
    I2S[bus_num].rate = rate;

    // calc integer and franctional for more precise timing
    // 
    uint8_t clkmInteger = clkmdiv;
    uint8_t clkmFraction = (clkmdiv - clkmInteger) * 63.0;

    // due to conf2.lcd_tx_wrx2_en being set for p8, bit rate is doubled
    // adjust by using bck here
#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)   
    uint8_t bck = (bits == 8 || bits == 16) ? 24 : 12;
#else
    // parallel modes on ESP32-S2 need higher rate (x4) to work properly e.g. producing 800KHz signal
    // it won't work (and ESP32-S2 becomes highly unstable/jumping into the bootloader mode) for present structure just by modyfing 'rate' like for ESP32
    // but it can be done other way by lowering tx_bck_div_num (bck) by 4   
    uint8_t bck = (bits == 8 || bits == 16) ? 6 : 12;
#endif

    i2sSetClock(bus_num, clkmInteger, clkmFraction, 63, bck, bits);

    return ESP_OK;
}



void IRAM_ATTR i2sDmaISR(void* arg)
{
    i2s_bus_t* i2s = (i2s_bus_t*)(arg);

    if (i2s->bus->int_st.out_eof) 
    {
 //       lldesc_t* item = (lldesc_t*)(i2s->bus->out_eof_des_addr);
        if (i2s->is_sending_data == I2s_Is_Pending)
        {
            i2s->is_sending_data = I2s_Is_Idle;
        }
        else if (i2s->is_sending_data == I2s_Is_Sending)
        {
            // loop the silent items
            lldesc_t* itemSilence = &i2s->dma_items[1];
            itemSilence->qe.stqe_next = &i2s->dma_items[0];

            i2s->is_sending_data = I2s_Is_Pending;
        }
    }

    i2s->bus->int_clr.val = i2s->bus->int_st.val;
}

size_t i2sWrite(uint8_t bus_num, uint8_t* data, size_t len, bool copy, bool free_when_sent) 
{
    if (bus_num >= I2S_NUM_MAX) 
    {
        return 0;
    }
    size_t blockSize = len;

    lldesc_t* item = &I2S[bus_num].dma_items[0]; 
    lldesc_t* itemsEnd = item + I2S[bus_num].dma_count;
    size_t dataLeft = len;
    uint8_t* pos = data;

    // skip front two silent items
    item += 2;

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

    // at the end of the data is the encoded silence
    uint8_t* posSilence = data + len - I2S_DMA_SILENCE_SIZE;

    // fix any extra blocks we have to point to silence
    while (item < itemsEnd)
    {
        item->buf = posSilence;
        item->size = I2S_DMA_SILENCE_SIZE;
        item->length = I2S_DMA_SILENCE_SIZE;

        item++;
    }

    // set silence item to silence 
    item = &I2S[bus_num].dma_items[0];
    item->buf = posSilence;
    item->size = I2S_DMA_SILENCE_SIZE;
    item->length = I2S_DMA_SILENCE_SIZE;
    item++;
    item->buf = posSilence;
    item->size = I2S_DMA_SILENCE_SIZE;
    item->length = I2S_DMA_SILENCE_SIZE;
    // and not loop
    item->qe.stqe_next = &I2S[bus_num].dma_items[2];

    I2S[bus_num].is_sending_data = I2s_Is_Sending;

    return len;
}

#endif // !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#endif // defined(ARDUINO_ARCH_ESP32) 

