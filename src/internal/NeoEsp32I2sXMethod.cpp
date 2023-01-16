#include <Arduino.h>
#include "NeoPixelBus.h"
#include "NeoEsp32I2sXMethod.h"

/*-------------------------------------------------------------------------

	High optimizated for speed using up to two parallel channels:
	  - FillBuffer: one time pass for better performance
	  - LUT tables support
	  - the i2s buffer prefill
	  - other optimization for i2s handling
	by @awawa-dev (https://github.com/awawa-dev)

-------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)

#if !defined(CONFIG_IDF_TARGET_ESP32S2)
	NeoEspI2sContext NeoEsp32I2s1Mux8Bus::s_context1 = NeoEspI2sContext(8);
#else
	NeoEspI2sContext NeoEsp32I2s0Mux8Bus::s_context0 = NeoEspI2sContext(8);
#endif

#endif

