#include <Arduino.h>
#include "ESP32Encoder.h"

#define MOTOR_TIMER_RATE 500 // ms

class Motor {
    public:
        Motor();
        void startTimer();
        int getPosition();
        void setPosition(int position);
    private:
        ESP32Encoder encoder;
        void timerCallback();

        int currentPosition;
        int targetPosition;
};