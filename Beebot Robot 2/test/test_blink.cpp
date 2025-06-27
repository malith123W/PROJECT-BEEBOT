#include <Arduino.h>
#define LED_PIN 2

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  Serial.println("LED ON");
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  Serial.println("LED OFF");
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}
