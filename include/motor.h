#include <Arduino.h>
#include "DRV8462.h"


class Motor {
    public:
        Motor();
        void init();
        void enable();
        void disable();
        int getPosition();
        void setSetpoint(int position);
        
    private:
        void startTimer();
        DRV8462 driver;
        void timerCallback();

        int currentPosition; // in units of steps
        float currentVelocity; // in units of steps/s
        float stepAccumulator; // accumulates fractional steps for sub-step precision
        static const int maxAcceleration = 100000; // max acceleration in steps/s^2
        static const int maxVelocity = 10000; // max velocity in steps/s
        int setpointPosition; // in units of steps
};