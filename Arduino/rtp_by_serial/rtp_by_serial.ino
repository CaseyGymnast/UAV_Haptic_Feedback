#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define HWSERIAL Serial1

Adafruit_DRV2605 drv;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
  Serial.println("DRV test");
  drv.begin();
    
  // Set Real-Time Playback mode
  drv.setMode(DRV2605_MODE_REALTIME);
}

uint8_t magnitude = 0;

void loop() {
  if (Serial.available()){
    magnitude = Serial.readString().toInt();
    Serial.print(magnitude);
  }

  if (magnitude < 256) {
    drv.setRealtimeValue(magnitude);
    delay(10);
  } else {
    drv.setRealtimeValue(0x00);
    delay(10);
    magnitude = 0;
  }
}
