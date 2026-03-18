#include <Arduino.h>
#include "DRV8462.h"
#include <string>


class Motor {
    public:
        Motor();
        void init();
        void enable();
        void disable();
        int getPosition();
        void setSetpoint(int position);
        std::string log();
        uint16_t getFault();
        void setHome(int homePosition);
        
    private:
        void startTimer();
        DRV8462 driver;
        void timerCallback();

        int currentPosition; // in units of steps
        float currentVelocity; // in units of steps/s
        float stepAccumulator; // accumulates fractional steps for sub-step precision
        static const int maxAcceleration = 80000; // max acceleration in steps/s^2
        static const int maxVelocity = 15000; // max velocity in steps/s
        int setpointPosition; // in units of steps
};