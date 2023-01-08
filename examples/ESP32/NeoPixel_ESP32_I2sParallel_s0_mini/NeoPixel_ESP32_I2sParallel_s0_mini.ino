
// NeoPixel_ESP32_I2sParallel - This example only works on the ESP32
// This sketch demonstrates the use of the I2S Parallel method allowing upto 8 hardware updated channels
// WARNING:  Currently underdevelopement and this sketch is used for testing/demonstration; do not consider it a best practices sketch.

#include <NeoPixelBus.h> 

const uint16_t PixelCount = 32; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t DebugPin = 5; // used for logic anaylyser trigger capture 

// moved construction into Setup() so we can capture log output into Serial
//
NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s0X8Sk6812Method>* strip0;
NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s0X8Sk6812Method>* strip1;
 
// focusing on the first four channels for now
//NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0X8Ws2812Method> strip4(PixelCount, PixelPin);
//NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0X8Ws2812Method> strip5(PixelCount, PixelPin);
//NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0X8Ws2812Method> strip6(PixelCount, PixelPin);
//NeoPixelBus<NeoGrbFeature, NeoEsp32I2s0X8Ws2812Method> strip7(PixelCount, PixelPin);
 
void setup() {
    Serial.begin(115200);
    //while (!Serial); // wait for serial attach
     
    Serial.println();
    Serial.println("Initializing...");
    //Serial.flush();

    strip0 = new NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s0X8Sk6812Method>(PixelCount, 12);
    strip1 = new NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s0X8Sk6812Method>(PixelCount, 13);
      
    strip0->Begin();
    Serial.println(" 1");
    strip1->Begin();
    Serial.println(" 2");
/*
//    pinMode(DebugPin, OUTPUT);
//    digitalWrite(DebugPin, LOW);*/
  
    Serial.println();
    Serial.println("Running...");
}
    
  
void loop() {
    delay(1000);

    Serial.println("On ...");
 
 

   
 
    //digitalWrite(DebugPin, HIGH);
    while(1)
    {
      for(uint8_t z=0; z<32; z++)
      {
          strip0->SetPixelColor(z, RgbColor(z,z,z));
          strip1->SetPixelColor(z, RgbColor(z,z,z));
      }
      strip0->Show();
      strip1->Show();
      int i=0;
      while(!strip0->CanShow() || !strip1->CanShow())
      {
        yield();
        if (i++>100)
        {
          Serial.print("Delayed CanShow");
          Serial.println(i);
          Serial.flush();
          delay(1);
        }
      }
      Serial.println("OK");
      Serial.flush();
    }
    
    
   
}
