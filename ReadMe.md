# About

I created this fork of Neopixelbus to improve LED sequence rendering for ESP32/ESP32-S2 devices using I2S interface.
It implements a double buffering technique using DMA buffer filling on demand, which means low memory consumption even when using a large number of RGB LEDs and low sequence rendering performance. It is used for single LED strip and multi-segment parallel output modes (up to 2 segments), which especially requires much more resources like memory and CPU, to prepare the DMA buffer. Without pre-filling on demand, you can experience a very long delay between frames or out of memory problem.

It is used in my HyperSerialESP32 and HyperSPI projects. Both ESP32 and ESP32-S2 (tested on Lolin mini) are supported.

# Usage

It's easy to use, because it still uses Neopixelbus API. Examples are available here: [link](https://github.com/awawa-dev/NeoPixelBus/tree/HyperSerialESP32/examples/ESP32)

# How on-demand prefill works?

On the following screenshot HyperHDR and ESP32 & ESP32-S2 with HyperSerialESP32 are handling 1800 RGBW LEDs at the same time (two 900LEDs segments). First two lines are for ESP32-S2 sending output to the two LEDs segments, the third line is rendering new frame busy status (or sleeping and waiting for DMA buffer to be freed), the fourth line is is awaken and filling part of DMA buffer that was already sent. Next four lines are for the second device: ESP32 and its two 450 RGBW LEDs segments. RGBW LEDs (sk6812) requires much more data to be stored in memory, prepared and sent than RGB like ws2812b.

![obraz](https://user-images.githubusercontent.com/69086569/215071097-97797592-4461-4c65-8c41-a6a24f0b30d5.png)

To enable such debugging define: `DebugBusyPin`, `DebugWorkingPin` before including this library and enable these pins for `OUTPUT` with `pinMode` afterwards.








