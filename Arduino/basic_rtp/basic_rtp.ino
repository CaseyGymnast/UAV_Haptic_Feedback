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

uint8_t rtp_index = 0;
//uint8_t rtp[] = {
//  0x30, 100, 0x32, 100, 
//  0x34, 100, 0x36, 100, 
//  0x38, 100, 0x3A, 100,
//  0x00, 100,
//  0x40, 200, 0x00, 100, 
//  0x40, 200, 0x00, 100, 
//  0x40, 200, 0x00, 100
//};

uint8_t rtp[] = {
  0x00, 0x30
};

void loop() {
  if (Serial.available()){
    rtp_index = Serial.readString().toInt();
  }
  if (rtp_index < sizeof(rtp)/sizeof(rtp[0])) {
    drv.setRealtimeValue(rtp[rtp_index]);
    delay(10);
  } else {
    drv.setRealtimeValue(0x00);
    delay(10);
    rtp_index = 0;
  }
  
}
