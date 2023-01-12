#pragma once

// ESP32C3/S3 I2S is not supported yet due to significant changes to interface
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "Esp32_i2s.h"

void i2sXInit(uint8_t bus_num, 
    uint32_t bits_per_sample, 
    uint32_t sample_rate, 
    i2s_tx_chan_mod_t chan_mod, 
    i2s_tx_fifo_mod_t fifo_mod, 
    size_t dma_count, 
    size_t dma_len);

void i2sXDeinit(uint8_t bus_num);

void i2sXSetPins(uint8_t bus_num, int8_t out, int8_t parallel, bool invert);

size_t i2sXWrite(uint8_t bus_num, uint8_t* data, size_t len, bool copy, bool free_when_sent);

bool i2sXWriteDone(uint8_t bus_num);

#ifdef __cplusplus
}
#endif

#endif
