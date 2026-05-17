#include <Arduino.h>
#include "controller.h"
#include "config.h"

/**
 * @brief Global controller instance.
 */
Controller controller;

/**
 * @brief Arduino setup entry point.
 */
void setup() {
  Serial.begin(115200);
  controller.init();
}

/**
 * @brief Arduino loop for periodic telemetry.
 */
void loop() {
  delay(50);
  Serial.println(controller.log().c_str());
  Serial.printf(">manual_mode:%d\n", analogRead(MANUAL_MODE_PIN));
  Serial.printf(">limit:%d\n", analogRead(LIMIT_SWITCH_PIN));
}
