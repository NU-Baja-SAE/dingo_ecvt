#include <Arduino.h>
#include "ESP32Encoder.h"


class Motor {
    public:
        Motor();
        void startTimer();
        int getPosition();
        void setSetpoint(int position);
    private:
        ESP32Encoder encoder;
        void timerCallback();

        int currentPosition;
        int setpointPosition;
};