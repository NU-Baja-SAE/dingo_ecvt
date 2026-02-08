#include <Arduino.h>
#include "controller.h"

Controller controller;

void setup() {
  Serial.begin(115200);
  controller.init();

}

void loop() {
  delay(1000);
}