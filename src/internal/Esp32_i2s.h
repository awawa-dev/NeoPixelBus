#pragma once

/*-------------------------------------------------------------------------

	High optimizated for speed using up to two parallel channels:
	  - FillBuffer: one time pass for better performance
	  - LUT tables support
	  - the i2s buffer prefill
	  - other optimization for i2s handling
	by @awawa-dev (https://github.com/awawa-dev)

-------------------------------------------------------------------------*/

// ESP32C3/S3 I2S is not supported yet due to significant changes to interface
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/semphr.h"
#include "esp_err.h"

#define I2S_DMA_MAX_DATA_LEN    4092// maximum bytes in one dma item

typedef enum {
	I2S_CHAN_STEREO, I2S_CHAN_RIGHT_TO_LEFT, I2S_CHAN_LEFT_TO_RIGHT, I2S_CHAN_RIGHT_ONLY, I2S_CHAN_LEFT_ONLY
} i2s_tx_chan_mod_t;

typedef enum {
	I2S_FIFO_16BIT_DUAL, I2S_FIFO_16BIT_SINGLE, I2S_FIFO_32BIT_DUAL, I2S_FIFO_32BIT_SINGLE
} i2s_tx_fifo_mod_t;

void i2sInit(uint8_t busIndex,
		uint32_t bitsPerSample,
		uint32_t sampleRate,
		i2s_tx_chan_mod_t chanMod,
		size_t dmaCount,
		bool parallel);

void i2sDeinit(uint8_t bus_num);

void i2sSetPins(uint8_t bus_num, int8_t out, int8_t parallel, bool invert);

size_t i2sWrite(uint8_t busIndex, uint8_t* data, size_t bufferSize, size_t dataLen);

bool i2sWriteDone(uint8_t bus_num);

uint8_t* i2sGetProcessedDataPtr(uint8_t bus_num);

xSemaphoreHandle i2sSemaphoreHandle(uint8_t bus_num);

#ifdef __cplusplus
}
#endif

#endif
