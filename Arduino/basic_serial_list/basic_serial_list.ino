#include <ArduinoJson.h>

#define HWSERIAL Serial1

String raw;
int up;
int y;


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop() {
  while (!Serial.available());
  raw = Serial.readStringUntil( '\n' );
  StaticJsonDocument<512> data;

  DeserializationError error = deserializeJson(data, raw);
  if (error) {
    Serial.println(error.c_str()); 
    return;
  }
  
  up = atoi(data["up"]);
  delay(20);
  
  Serial.print(up);
}
