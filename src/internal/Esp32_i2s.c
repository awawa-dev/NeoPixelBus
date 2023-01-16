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

	High optimizated for speed using up to two parallel channels:
	  - FillBuffer: one time pass for better performance
	  - LUT tables support
	  - the i2s buffer prefill
	  - other optimization for i2s handling
	by @awawa-dev (https://github.com/awawa-dev)

-------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_ESP32)

#include "sdkconfig.h" // this sets useful config symbols, like CONFIG_IDF_TARGET_ESP32C3

// ESP32 C3 I2S is not supported yet due to significant changes to interface
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

#define MATRIX_DETACH_OUT_SIG 0x100

#if ESP_IDF_VERSION_MAJOR<=4
	#define I2S_BASE_CLK (160000000L)
#endif

#define I2S_DMA_BLOCK_COUNT_DEFAULT			16
#define I2S_DMA_SILENCE_BLOCK_FRONT_COUNT	2

typedef struct
{
	i2s_dev_t* bus;
	intr_handle_t isrHandler;
	lldesc_t* dmaItems;
	size_t dmaLen;
	volatile bool isReady;
	uint8_t* dataPtr;
	volatile uint8_t* lastDataPtr;
	xSemaphoreHandle i2sXSemaphore;
	uint32_t frameSig1;
	uint32_t frameSig2;
	size_t backBuffer;
} i2s_bus_t;

static uint8_t i2sSilenceBuf[16] = { 0 };

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
static i2s_bus_t I2SX[I2S_NUM_MAX] =
{
	{&I2S0, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, true, NULL, NULL, NULL, 0, 0, 0},
	{&I2S1, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, true, NULL, NULL, NULL, 0, 0, 0}
};
#else
static i2s_bus_t I2SX[I2S_NUM_MAX] =
{
	{&I2S0, NULL, NULL, I2S_DMA_BLOCK_COUNT_DEFAULT, true, NULL, NULL, NULL, 0, 0, 0}
};
#endif

xSemaphoreHandle i2sSemaphoreHandle(uint8_t busIndex)
{
	if (busIndex >= I2S_NUM_MAX)
	{
		return NULL;
	}

	return I2SX[busIndex].i2sXSemaphore;
}

bool i2sInitDmaItems(uint8_t busIndex, size_t _dmaCount, bool parallel)
{
	I2SX[busIndex].backBuffer = (parallel) ? 4 : 2;
	I2SX[busIndex].dmaLen = _dmaCount +  I2S_DMA_SILENCE_BLOCK_FRONT_COUNT + I2SX[busIndex].backBuffer + 1;

	size_t dmaCount = I2SX[busIndex].dmaLen;

	I2SX[busIndex].dmaItems = (lldesc_t*)heap_caps_malloc(dmaCount * sizeof(lldesc_t), MALLOC_CAP_DMA);
	if (I2SX[busIndex].dmaItems == NULL)
	{
		log_e("MEM ERROR!");
		return false;
	}

	memset(I2SX[busIndex].dmaItems, 0, dmaCount * sizeof(lldesc_t));

	for(int i=0; i< dmaCount; i++)
	{
		lldesc_t* item = &I2SX[busIndex].dmaItems[i];
		item->owner = 1;
		item->eof = (i >= I2S_DMA_SILENCE_BLOCK_FRONT_COUNT);
		item->buf = (volatile unsigned char *)&i2sSilenceBuf;
		item->size = sizeof(i2sSilenceBuf)/((parallel)? 1: 4);
		item->length = item->size;
		item->qe.stqe_next = &I2SX[busIndex].dmaItems[(i+1) % dmaCount];		
	}

	I2SX[busIndex].dmaItems[1].qe.stqe_next = &(I2SX[busIndex].dmaItems[0]);

	return true;
}

void i2sResetFifoDMA(i2s_dev_t* i2s)
{
	i2s->lc_conf.in_rst=1; i2s->lc_conf.out_rst=1; i2s->lc_conf.ahbm_rst=1; i2s->lc_conf.ahbm_fifo_rst=1;
	i2s->lc_conf.in_rst=0; i2s->lc_conf.out_rst=0; i2s->lc_conf.ahbm_rst=0; i2s->lc_conf.ahbm_fifo_rst=0;
	i2s->conf.tx_reset=1; i2s->conf.tx_fifo_reset=1; i2s->conf.rx_fifo_reset=1;
	i2s->conf.tx_reset=0; i2s->conf.tx_fifo_reset=0; i2s->conf.rx_fifo_reset=0;
}

bool i2sDeinitDmaItems(uint8_t busIndex)
{
	if (busIndex >= I2S_NUM_MAX || I2SX[busIndex].dmaItems == NULL)
	{
		return false;
	}

	vTaskDelay(2);

	esp_intr_disable(I2SX[busIndex].isrHandler);

	heap_caps_free(I2SX[busIndex].dmaItems);
	I2SX[busIndex].dmaItems = NULL;

	vSemaphoreDelete(I2SX[busIndex].i2sXSemaphore);
	I2SX[busIndex].i2sXSemaphore = NULL;

	I2SX[busIndex].bus->out_link.stop = 1;
	I2SX[busIndex].bus->conf.tx_start = 0;
	I2SX[busIndex].bus->int_ena.out_eof = 0;
	I2SX[busIndex].bus->int_ena.out_dscr_err = 0;

	I2SX[busIndex].bus->int_clr.val = I2SX[busIndex].bus->int_st.val;

	i2sResetFifoDMA(I2SX[busIndex].bus);

	esp_intr_free(I2SX[busIndex].isrHandler);
	I2SX[busIndex].isrHandler = NULL;

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
	if (busIndex == 1)
		periph_module_disable(PERIPH_I2S1_MODULE);
	else
#endif
	if (busIndex == 0)
		periph_module_disable(PERIPH_I2S0_MODULE);

	return true;
}

void i2sDeinit(uint8_t busIndex)
{
	i2sDeinitDmaItems(busIndex);
}

void i2sSetPins(uint8_t busIndex, int8_t out, int8_t parallel, bool invert)
{
	if (busIndex >= I2S_NUM_MAX)
	{
		return;
	}

	if (out >= 0)
	{
		pinMode(out, OUTPUT);

		uint32_t i2sSignal;

		#if !defined(CONFIG_IDF_TARGET_ESP32S2)
		if (busIndex == 1)
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

bool i2sWriteDone(uint8_t busIndex)
{
	if (busIndex >= I2S_NUM_MAX)
	{
		return false;
	}
	return (bool)(I2SX[busIndex].isReady);
}

uint8_t* i2sGetProcessedDataPtr(uint8_t busIndex)
{
	if (busIndex >= I2S_NUM_MAX)
	{
		return NULL;
	}

	return (uint8_t*)I2SX[busIndex].lastDataPtr;
}

esp_err_t i2sSetSampleRate(uint8_t busIndex, uint32_t rate, uint8_t bits, bool parallel)
{
	// parallel modes need higher frequency on esp32
	double clkmdiv = (double)I2S_BASE_CLK / ((rate * 384) + 1);
	uint8_t clkmInteger = clkmdiv;
	uint8_t clkmFractionA = 63;
	uint8_t clkmFractionB = (clkmdiv - clkmInteger) * 63.0;
	uint8_t bck = (parallel && busIndex == 0) ? 6 : 12;

	i2s_dev_t* i2s = I2SX[busIndex].bus;

	i2s->clkm_conf.val = 0;
	i2s->clkm_conf.clkm_div_a = clkmFractionA;
	i2s->clkm_conf.clkm_div_b = clkmFractionB;
	i2s->clkm_conf.clkm_div_num = clkmInteger;
#if !defined(CONFIG_IDF_TARGET_ESP32S2)
	i2s->clkm_conf.clka_en = 0;
#else
	i2s->clkm_conf.clk_sel = 2;
#endif
	i2s->sample_rate_conf.tx_bck_div_num = bck;
	i2s->sample_rate_conf.tx_bits_mod = bits;

	return ESP_OK;
}

void IRAM_ATTR i2sXDmaISR(void* arg)
{
	i2s_bus_t* i2s = (i2s_bus_t*)(arg);

	if (i2s->bus->int_st.out_eof)
	{
		lldesc_t* item = (lldesc_t*)(i2s->bus->out_eof_des_addr);

		for(int found = I2S_DMA_SILENCE_BLOCK_FRONT_COUNT; found < i2s->dmaLen; found++)
			if (item == &(i2s->dmaItems[found]))
			{
				lldesc_t* itemSilence = &i2s->dmaItems[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT - 1];
				itemSilence->qe.stqe_next = &i2s->dmaItems[0];

				if (found >= i2s->dmaLen - i2s->backBuffer)
				{
					if (i2s->frameSig2 == i2s->frameSig1)
					{
						i2s->isReady = true;
						i2s->lastDataPtr = NULL;
						i2s->frameSig2 = 0;
					}
					else
						break;
				}
				else
				{
					i2s->frameSig2 = i2s->frameSig1;

					int memPos = found - I2S_DMA_SILENCE_BLOCK_FRONT_COUNT;
					if (memPos > 0)
					{
						i2s->lastDataPtr = i2s->dataPtr + memPos * I2S_DMA_MAX_DATA_LEN;
					}
					else
						break;
				}

				portBASE_TYPE isWakeUp = 0;
				xSemaphoreGiveFromISR( i2s->i2sXSemaphore, &isWakeUp );
				if(isWakeUp == pdTRUE) portYIELD_FROM_ISR();

				break;
			}
	}

	i2s->bus->int_clr.val = i2s->bus->int_st.val;
}

void i2sInit(uint8_t busIndex,
			uint32_t bitsPerSample,
			uint32_t sampleRate,
			i2s_tx_chan_mod_t chanMod,
			size_t dmaCount,
			bool parallel)
{
	if (busIndex >= I2S_NUM_MAX || I2SX[busIndex].dmaItems != NULL)
	{
		return;
	}

	assert(I2S_DMA_SILENCE_BLOCK_FRONT_COUNT >= 2);

	if (!i2sInitDmaItems(busIndex, dmaCount, parallel))
	{
		return;
	}

	I2SX[busIndex].i2sXSemaphore = xSemaphoreCreateBinary();

	xSemaphoreGive(I2SX[busIndex].i2sXSemaphore);

	#if !defined(CONFIG_IDF_TARGET_ESP32S2)
	if (busIndex)
	{
		periph_module_enable(PERIPH_I2S1_MODULE);
	}
	else
	#endif
	{
		periph_module_enable(PERIPH_I2S0_MODULE);
	}

	i2s_dev_t* i2s = I2SX[busIndex].bus;
	i2s->out_link.stop = 1;
	i2s->conf.tx_start = 0;
	i2s->int_ena.val = 0;
	i2s->int_clr.val = 0xFFFFFFFF;
	i2s->fifo_conf.dscr_en = 0;

	i2sResetFifoDMA(i2s);

	i2s->conf2.val = 0;
	i2s->conf2.lcd_en = (parallel);
	i2s->lc_conf.val = 0;
	i2s->lc_conf.out_eof_mode = 1;
	i2s->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
	i2s->conf_chan.val = 0;
	i2s->conf_chan.tx_chan_mod = chanMod;
	i2s->conf.val = 0;
	i2s->timing.val = 0;
	i2s->conf1.tx_pcm_bypass = 1;
	i2s->fifo_conf.val = 0;
	i2s->fifo_conf.tx_fifo_mod_force_en = parallel;
	i2s->fifo_conf.tx_fifo_mod = 1;
	i2s->fifo_conf.tx_data_num = 16;

	i2sSetSampleRate(busIndex, sampleRate, bitsPerSample, parallel);

	//  enable intr in cpu
	int i2sIntSource = ETS_I2S0_INTR_SOURCE;

	#if !defined(CONFIG_IDF_TARGET_ESP32S2)
		i2sIntSource = (busIndex == 1) ? ETS_I2S1_INTR_SOURCE : ETS_I2S0_INTR_SOURCE;
	#endif

	esp_intr_alloc(i2sIntSource,
					ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
					&i2sXDmaISR, &I2SX[busIndex],
					&I2SX[busIndex].isrHandler);

	//  enable send intr
	i2s->int_ena.out_eof = 1;
	i2s->int_ena.out_dscr_err = 1;

	// reset FIFO/DMA
	i2sResetFifoDMA(i2s);

	// Enable and configure DMA
	i2s->fifo_conf.dscr_en = 1;
	i2s->out_link.start = 0;
	i2s->out_link.addr = (uint32_t)(&I2SX[busIndex].dmaItems[0]);
	i2s->out_link.start = 1;
	i2s->conf.tx_start = 1;

	esp_intr_enable(I2SX[busIndex].isrHandler);
}

size_t i2sWrite(uint8_t busIndex, uint8_t* data, size_t bufferSize, size_t dataLen)
{
	if (busIndex >= I2S_NUM_MAX || I2SX[busIndex].dmaItems == NULL)
	{
		return 0;
	}

	lldesc_t *item = &I2SX[busIndex].dmaItems[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT], *sItem;
	size_t dataLeft = dataLen;
	uint8_t* pos = data;

	esp_intr_disable(I2SX[busIndex].isrHandler);
	
	I2SX[busIndex].bus->int_clr.val = I2SX[busIndex].bus->int_st.val;

	I2SX[busIndex].frameSig1++;
	I2SX[busIndex].dataPtr = data;
	I2SX[busIndex].lastDataPtr = NULL;
	I2SX[busIndex].isReady = false;

	while (dataLeft)
	{
		size_t blockSize = (dataLeft < I2S_DMA_MAX_DATA_LEN) ? dataLeft : I2S_DMA_MAX_DATA_LEN;

		item++;
		item->buf = pos;
		item->size = blockSize;
		item->length = blockSize;

		pos += blockSize;
		dataLeft -= blockSize;
	}

	if (bufferSize > dataLen + 4)
	{		
		int sSize =  bufferSize - dataLen;
		sSize -= sSize % 4;
		sItem = &I2SX[busIndex].dmaItems[I2SX[busIndex].dmaLen - I2SX[busIndex].backBuffer];
		sItem->buf = data + (bufferSize - sSize);
		sItem->size = sSize;
		sItem->length = sSize;
		item->qe.stqe_next = sItem;
	}

	esp_intr_enable(I2SX[busIndex].isrHandler);

	I2SX[busIndex].dmaItems[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT-1].qe.stqe_next = &I2SX[busIndex].dmaItems[I2S_DMA_SILENCE_BLOCK_FRONT_COUNT];

	return bufferSize;
}

#endif // !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#endif // defined(ARDUINO_ARCH_ESP32)

