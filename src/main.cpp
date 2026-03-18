#include <Arduino.h>
#include "controller.h"
#include "config.h"

Controller controller;

void setup() {
  Serial.begin(115200);
  controller.init();
}

void loop() {
  delay(50);
  Serial.println(controller.log().c_str());
  Serial.println(digitalRead(LIMIT_SWITCH_PIN));
}