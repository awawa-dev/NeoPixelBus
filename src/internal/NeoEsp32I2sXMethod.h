#pragma once

/*-------------------------------------------------------------------------
NeoPixel library helper functions for Esp32.

Written by Michael C. Miller.

I invest time and resources providing this open source code,
please support me by dontating (see https://github.com/Makuna/NeoPixelBus)

-------------------------------------------------------------------------
This file is part of the Makuna/NeoPixelBus library.

NeoPixelBus is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or (at your option) any later version.

NeoPixelBus is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with NeoPixel.  If not, see
<http://www.gnu.org/licenses/>.
-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------

    High optimizated for speed using two parallel channels:
      - FillBuffer: one time pass for better performance
      - LUT tables support
      - other optimization for i2s handling   
    by @awawa-dev (https://github.com/awawa-dev)

-------------------------------------------------------------------------*/

extern "C"
{
#include <Arduino.h>
#include "Esp32_i2sX.h"
}


#pragma once

// ESP32C3/S3 I2S is not supported yet due to significant changes to interface
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

class NeoEspI2sContext
{
public:
    const static size_t DmaBytesPerPixelBits = 4;
    const uint8_t BusMaxCount;
    const static uint8_t InvalidMuxId = -1;

    uint32_t I2sBufferSize; // total size of I2sBuffer
    uint32_t* I2sBuffer;    // holds the DMA buffer that is referenced by I2sBufDesc

    size_t MaxBusDataSize; // max size of stream data from any single mux bus
    uint32_t UpdateMap;     // bitmap flags of mux buses to track update state
    uint32_t UpdateMapMask; // mask to used bits in s_UpdateMap
    uint8_t BusCount;      // count of mux buses

    uint8_t i2sBus;
    uint8_t* pixelMuxData[8] = {};
    size_t pixelMuxDataLen[8] = {};

    const uint64_t EncodedZeroBit64 = 0x0000000000000001;
    const uint64_t EncodedOneBit64 = 0x0000000100010001;
    const uint32_t EncodedZeroBit = 0x00010000;
    const uint32_t EncodedOneBit = 0x01010001;  

    uint64_t signalLut64[4]=
    {
        (((0) ? EncodedOneBit64 : EncodedZeroBit64) << (1)) | (((0) ? EncodedOneBit64 : EncodedZeroBit64) << (0)),
        (((0) ? EncodedOneBit64 : EncodedZeroBit64) << (1)) | (((1) ? EncodedOneBit64 : EncodedZeroBit64) << (0)),
        (((1) ? EncodedOneBit64 : EncodedZeroBit64) << (1)) | (((0) ? EncodedOneBit64 : EncodedZeroBit64) << (0)),
        (((1) ? EncodedOneBit64 : EncodedZeroBit64) << (1)) | (((1) ? EncodedOneBit64 : EncodedZeroBit64) << (0))
    };

    uint64_t signalLut32[16] = {};

    NeoEspI2sContext(uint8_t _BusMaxCount) :
        BusMaxCount(_BusMaxCount),
        I2sBufferSize(0),
        I2sBuffer(nullptr),
        MaxBusDataSize(0),
        UpdateMap(0),
        UpdateMapMask(0),
        BusCount(0),
        i2sBus(0)
    {
        for(uint8_t index2 = 0; index2 < 4; index2++)
            for(uint8_t index1 = 0; index1 < 4; index1++)
            {
                uint8_t value1  = index1 << 6;
                uint8_t value2  = index2 << 6;
                uint8_t targetIndex = (index2 << 2) | index1;
                uint32_t* pDma32 = reinterpret_cast<uint32_t*>(&(signalLut32[targetIndex]));

                for (uint8_t i = 0; i < 2; i++)
                {
                    uint32_t dma32 = (((value1 & 0x80) ? EncodedOneBit : EncodedZeroBit) << (0));
                    dma32 |= (((value2 & 0x80) ? EncodedOneBit : EncodedZeroBit) << (1));
                    *(pDma32++) = dma32;
                    value1 <<= 1;
                    value2 <<= 1;
                }
            }
    }

    uint8_t RegisterNewMuxBus(const size_t dataSize)
    {
        // find first available bus id
        uint8_t muxId = 0;
        while (muxId < BusMaxCount)
        {
            uint32_t muxIdField = (1 << muxId);
            if ((UpdateMapMask & muxIdField) == 0)
            {
                // complete registration
                BusCount++;
                UpdateMapMask |= muxIdField;
                if (dataSize > MaxBusDataSize)
                {
                    MaxBusDataSize = dataSize;
                }
                break;
            }
            muxId++;
        }
        return muxId;
    }

    bool DeregisterMuxBus(uint8_t muxId)
    {
        uint32_t muxIdField = (1 << muxId);
        if (UpdateMapMask & muxIdField)
        {
            // complete deregistration
            BusCount--;
            UpdateMapMask &= ~muxIdField;
            if (UpdateMapMask == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool IsAllMuxBusesUpdated()
    {
        return (UpdateMap == UpdateMapMask);
    }

    void MarkMuxBusUpdated(uint8_t muxId)
    {
        UpdateMap |= (1 << muxId);
    }

    void ResetMuxBusesUpdated()
    {
        UpdateMap = 0;
    }

    void Construct(const uint8_t busNumber, uint32_t i2sSampleRate, uint8_t muxId, uint8_t* data, size_t dataLen)
    {
        i2sBus = busNumber;
        pixelMuxData[muxId] = data;
        pixelMuxDataLen[muxId] = dataLen;

        // construct only once on first time called
        if (I2sBuffer == nullptr)
        {
            // $REVIEW this change to evaluate for cleanup. Its best to keep branching like this to a single location and wrap the concept into a const like the original.
            // 1 byte on the input => 8 (bits) * DmaBytesPerPixelBits (bits of 4 bytes are used for neopixel) * only x8 on I2S1 is true 8bits mode and requires less memory than 16bits modes
            I2sBufferSize = MaxBusDataSize * 8 * DmaBytesPerPixelBits * ((busNumber == 1 && BusMaxCount == 8) ? 1 : 2);

            // must have a 8 byte aligned buffer for i2s
            uint32_t alignment = I2sBufferSize % 8;
            if (alignment)
            {
                I2sBufferSize += 8 - alignment;
            }

            size_t dmaBlockCount = (I2sBufferSize + I2S_DMA_MAX_DATA_LEN - 1) / I2S_DMA_MAX_DATA_LEN;

            i2sXInit(busNumber,
                BusMaxCount,
                i2sSampleRate,
                I2S_CHAN_RIGHT_TO_LEFT,
                I2S_FIFO_32BIT_SINGLE,
                dmaBlockCount,
                0);

            I2sBuffer = static_cast<uint32_t*>(heap_caps_malloc(I2sBufferSize, MALLOC_CAP_DMA));
            memset(I2sBuffer, 0x00, I2sBufferSize);
        }
    }

    void Destruct(const uint8_t busNumber)
    {
        if (I2sBuffer == nullptr)
        {
            return;
        }

        i2sXSetPins(busNumber, -1, -1, false);
        i2sXDeinit(busNumber);

        heap_caps_free(I2sBuffer);

        I2sBufferSize = 0;
        I2sBuffer = nullptr;
        MaxBusDataSize = 0;
        UpdateMap = 0;
        UpdateMapMask = 0;
        BusCount = 0;
    }

    void FillBuffers()
    {
        uint64_t* pDma64 = reinterpret_cast<uint64_t*>(I2sBuffer);

        const uint8_t* data1 = pixelMuxData[0];
        const uint8_t* end1 = data1 + pixelMuxDataLen[0];
        const uint8_t* data2 = pixelMuxData[1];
        const uint8_t* end2 = data2 + pixelMuxDataLen[1];

        for(; data1 < end1 || data2 < end2 ; data1++, data2++)
        {
            uint8_t value1 = (data1 < end1) ? *data1 : 0;
            uint8_t value2 = (data2 < end2) ? *data2 : 0;

            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if (i2sBus == 0)
                {
                    *(pDma64++) = signalLut64[((value2 & 0x80) ? 2 :0) | ((value1 & 0x80) ? 1 :0) ];
                    value1 <<= 1;
                    value2 <<= 1;
                }
                else
                {
                    *(pDma64++) = signalLut32[((value2 & 0xc0) >> 4) | ((value1 & 0xc0) >> 6)];
                    value1 <<= 2;
                    value2 <<= 2;
                    bit++;
                }
            }
        }
    }
};

class NeoEsp32I2sMuxBus
{
public:
    NeoEsp32I2sMuxBus(uint8_t i2sBusNumber, NeoEspI2sContext& context) :
        _i2sBusNumber(i2sBusNumber),
        _context(context),
        _muxId(NeoEspI2sContext::InvalidMuxId)
    {
    }

    void RegisterNewMuxBus(size_t dataSize)
    {
        _muxId = _context.RegisterNewMuxBus(dataSize);
    }

    void Initialize(uint8_t pin, uint32_t i2sSampleRate, bool invert, uint8_t* _data, size_t dataLen)
    {
        _context.Construct(_i2sBusNumber, i2sSampleRate, _muxId, _data, dataLen);
        i2sXSetPins(_i2sBusNumber, pin, _muxId, invert);
    }

    void DeregisterMuxBus()
    {
        if (_context.DeregisterMuxBus(_muxId))
        {
            _context.Destruct(_i2sBusNumber);
        }
        // disconnect muxed pin?
        _muxId = NeoEspI2sContext::InvalidMuxId;
    }

    void StartWrite()
    {
        if (_context.IsAllMuxBusesUpdated())
        {
            _context.ResetMuxBusesUpdated();
            _context.FillBuffers();
            i2sXWrite(_i2sBusNumber, reinterpret_cast<uint8_t*>(_context.I2sBuffer), _context.I2sBufferSize, false, false);
        }
    }

    bool IsWriteDone() const
    {
        return i2sXWriteDone(_i2sBusNumber);
    }

    void MarkUpdated()
    {
        _context.MarkMuxBusUpdated(_muxId);
    }

private:
    const uint8_t _i2sBusNumber;
    NeoEspI2sContext& _context;
    uint8_t _muxId;
};

#if defined(CONFIG_IDF_TARGET_ESP32S2)
    class NeoEsp32I2s0Mux8Bus : public NeoEsp32I2sMuxBus
    {
    public:
        NeoEsp32I2s0Mux8Bus() : NeoEsp32I2sMuxBus(0, s_context0)
        {
        }
    private:
        static NeoEspI2sContext s_context0;
    };
#else
    class NeoEsp32I2s1Mux8Bus : public NeoEsp32I2sMuxBus
    {
    public:
        NeoEsp32I2s1Mux8Bus() : NeoEsp32I2sMuxBus(1, s_context1)
        {
        }
    private:
        static NeoEspI2sContext s_context1;
    };
#endif

template<typename T_SPEED, typename T_BUS, typename T_INVERT> class NeoEsp32I2sXMethodBase
{
public:
    typedef NeoNoSettings SettingsObject;

    NeoEsp32I2sXMethodBase(uint8_t pin, uint16_t pixelCount, size_t elementSize, size_t settingsSize) :
        _sizeData(pixelCount * elementSize + settingsSize),
        _pin(pin)
    {
        _bus.RegisterNewMuxBus(_sizeData + T_SPEED::ResetTimeUs / T_SPEED::ByteSendTimeUs);
    }

    ~NeoEsp32I2sXMethodBase()
    {
        while (!_bus.IsWriteDone())
        {
            yield();
        }

        _bus.DeregisterMuxBus();

        free(_data);
    }

    bool IsReadyToUpdate() const
    {
        return _bus.IsWriteDone();
    }

    void Initialize()
    {
        _data = static_cast<uint8_t*>(malloc(_sizeData));
        _bus.Initialize(_pin, T_SPEED::I2sSampleRate, T_INVERT::Inverted, _data, _sizeData);
    }

    void Update(bool)
    {
        // wait for not actively sending data
        while (!_bus.IsWriteDone())
        {
            yield();
        }

        _bus.MarkUpdated();
        _bus.StartWrite(); // only triggers actual write after all mux busses have updated
    }

    void MarkUpdated()
    {
        _bus.MarkUpdated();
    }

    uint8_t* getData() const
    {
        return _data;
    };

    size_t getDataSize() const
    {
        return _sizeData;
    }

    void applySettings([[maybe_unused]] const SettingsObject& settings)
    {
    }

private:
    const size_t  _sizeData;    // Size of '_data' buffer 
    const uint8_t _pin;         // output pin number

    T_BUS _bus;          // holds instance for mux bus support
    uint8_t* _data;      // Holds LED color values
};

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
    typedef NeoEsp32I2sXMethodBase<NeoEsp32I2sSpeedWs2812x, NeoEsp32I2s1Mux8Bus, NeoEsp32I2sNotInverted> NeoEsp32I2s1X8Ws2812Method;
    typedef NeoEsp32I2sXMethodBase<NeoEsp32I2sSpeedSk6812, NeoEsp32I2s1Mux8Bus, NeoEsp32I2sNotInverted> NeoEsp32I2s1X8Sk6812Method;
#else
    typedef NeoEsp32I2sXMethodBase<NeoEsp32I2sSpeedWs2812x, NeoEsp32I2s0Mux8Bus, NeoEsp32I2sNotInverted> NeoEsp32I2s0X8Ws2812Method;
    typedef NeoEsp32I2sXMethodBase<NeoEsp32I2sSpeedSk6812, NeoEsp32I2s0Mux8Bus, NeoEsp32I2sNotInverted> NeoEsp32I2s0X8Sk6812Method;
#endif

#endif