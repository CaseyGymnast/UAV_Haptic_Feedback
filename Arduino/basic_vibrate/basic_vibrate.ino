/*
Basic timer circuit to trigger haptic motor controller at a specified interval.
Uses Flora and DRV2605L Haptic Motor Controller
Tutorial: https://learn.adafruit.com/haptic-headband
*/

#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;
int pin_SDA = 18; 
int pin_SCL = 19; 

uint8_t effect = 118;  // 1 Pulse and 118 Buzz
const uint32_t wait = 2; // Time between reminders, in seconds

void setup() {
  Serial.begin(9600);
  Wire.setSDA(pin_SDA);
  Wire.setSCL(pin_SCL);
  drv.begin();
//  drv.useLRA();
  
  drv.selectLibrary(6);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 
  
}

void loop() {
  Serial.print("Effect #"); Serial.println(effect);

  // set the effect to play
  drv.setWaveform(0, effect);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();
    
  delay(wait * 1000);

//  effect++;
//  if (effect > 123) effect = 1;

}
