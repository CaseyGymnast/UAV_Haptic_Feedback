#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include <ArduinoJson.h>

#define HWSERIAL Serial1
#define TCAADDR 0x70

Adafruit_DRV2605 drv0;  // Up
Adafruit_DRV2605 drv1;  // Right
Adafruit_DRV2605 drv2;  // Down
Adafruit_DRV2605 drv3;  // Left

Adafruit_DRV2605 drv_arr[4] = {drv0, drv1, drv2, drv3};

uint8_t magnitude = 0x70;
String keys[4] = {"up", "right", "down", "left"};
String raw;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  delay(1000);
  
  Wire.begin();
  Serial.begin(9600);
  Serial.setTimeout(1);

  for (uint8_t i; i < 4; i++) {
    // Select I2C Bus 0
    tcaselect(i);
    
    // Initialize DRV2065 and Set to Real Time
    drv_arr[i].begin();
    drv_arr[i].setMode(DRV2605_MODE_REALTIME);
  }  
}

void loop() {
  tcaselect(0);
  drv_arr[0].setRealtimeValue(magnitude);
  delay(1000);
  drv_arr[0].setRealtimeValue(0x00);
  delay(1000);
  
  tcaselect(1);
  drv_arr[1].setRealtimeValue(magnitude);
  delay(1000);
  drv_arr[1].setRealtimeValue(0x00);
  delay(1000);


  tcaselect(2);
  drv_arr[2].setRealtimeValue(magnitude);
  delay(1000);
  drv_arr[2].setRealtimeValue(0x00);
  delay(1000);
  
  tcaselect(3);
  drv_arr[3].setRealtimeValue(magnitude);
  delay(1000);
  drv_arr[3].setRealtimeValue(0x00);
  delay(1000);

  magnitude = 0x00;
}
