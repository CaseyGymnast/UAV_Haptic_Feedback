#define HWSERIAL Serial1

int x;
int y;


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop() {
  while (!Serial.available());
  x = Serial.readString().toInt();
  y = Serial.readString().toInt();
  Serial.print(x + y);
}
