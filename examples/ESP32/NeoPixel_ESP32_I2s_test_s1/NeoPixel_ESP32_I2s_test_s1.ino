// NeoPixel_ESP32_I2sParallel - This example only works on the ESP32
// This sketch demonstrates the use of the new I2S method
// WARNING:  Currently underdevelopement and this sketch is used for testing/demonstration; do not consider it a best practices sketch.

#define DebugWorkingPin 16 // used for logic anaylyser trigger capture & performance analysis (time measurement)
#define DebugBusyPin 17 // used for logic anaylyser trigger capture & performance analysis (time measurement)

#include <NeoPixelBus.h> 

const uint16_t PixelCount = 450; // test performance for two 450 RGBW pixels parallel segments

NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s1Sk6812Method>* strip0;
 
void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.println(); 
    Serial.println("Initializing...");
    //Serial.flush();
    delay(2000);
    strip0 = new NeoPixelBus<NeoGrbwFeature, NeoEsp32I2s1Sk6812Method>(PixelCount, 2);
    pinMode(DebugWorkingPin, OUTPUT);
    pinMode(DebugBusyPin, OUTPUT);    
    Serial.println("Begins...");
    strip0->Begin();
  
    Serial.println();
    Serial.println("Running...");
}
    
void loop() {
    delay(1000);

    Serial.println("On ...");
     
    strip0->SetPixelColor(0, RgbwColor(255, 200, 200, 255));    
    for(int i = 1; i < PixelCount; i++)
    {
        strip0->SetPixelColor(i, RgbColor(i % 255));
    }
    strip0->SetPixelColor(PixelCount - 1, RgbwColor(255, 255, 255, 255));
    
    digitalWrite(DebugBusyPin, HIGH);
    strip0->Show();
    digitalWrite(DebugBusyPin, LOW);

    strip0->SetPixelColor(0, RgbColor(0));
    
    digitalWrite(DebugBusyPin, HIGH);
    while(!strip0->CanShow())
        yield();
    digitalWrite(DebugBusyPin, LOW);  

    strip0->Show();

    Serial.println("OFF");
}
