#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define HWSERIAL Serial1
#define TCAADDR 0x70

Adafruit_DRV2605 drv0;
Adafruit_DRV2605 drv1;
Adafruit_DRV2605 drv2;
Adafruit_DRV2605 drv3;

Adafruit_DRV2605 drv_arr[2] = {drv0, drv1};

int pin_SDA = 18; 
int pin_SCL = 19;

uint8_t magnitude = 0x30;
uint8_t disc = 0;

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

  for (uint8_t i; i < 2; i++) {
    // Select I2C Bus 0
    tcaselect(i);
    
    // Initialize DRV2065 and Set to Real Time
    drv_arr[i].begin();
    drv_arr[i].setMode(DRV2605_MODE_REALTIME);

    Serial.println("Initialized");
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
}
