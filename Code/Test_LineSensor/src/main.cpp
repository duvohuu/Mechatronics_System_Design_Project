#include <Arduino.h>


void setup() {
  Serial.begin(9600);
  pinMode(A0, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A8);
  Serial.println(sensorValue);
  delay(1000);
}

