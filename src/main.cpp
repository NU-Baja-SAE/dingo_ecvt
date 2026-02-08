#include <Arduino.h>
#include "controller.h"

Controller controller;

void setup() {
  Serial.begin(115200);
  controller.startTimer();

}

void loop() {
  Serial.println("Hello, world!");
  delay(1000);
}