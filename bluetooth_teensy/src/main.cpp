#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  // From BT to USB
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // From USB to BT
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }
}