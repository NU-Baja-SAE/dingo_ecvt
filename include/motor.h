#include <Arduino.h>
#include "DRV8462.h"
#include "encoder.h"
#include <string>


class Motor {
    public:
        Motor();
        void init();
        void enable();
        void disable();
        int getPosition();
        int getSetpoint();
        void setPosition(int position);
        void setSetpoint(int position);
        std::string log();
        uint16_t getFault();
        void setHome(int homePosition);
        
    private:
        void startTimer();
        DRV8462 driver;
        Encoder encoder;
        void timerCallback();

        int currentPosition; // in units of steps
        int lastPosition; // in units of steps, used to calculate velocity
        float currentVelocity; // in units of steps/s
        float stepAccumulator; // accumulates fractional steps for sub-step precision
        // static const int maxAcceleration = 5000; // max acceleration in steps/s^2
        // static const int maxVelocity = 1000; // max velocity in steps/s
        static const int maxAcceleration = 60000; // max acceleration in steps/s^2
        static const int maxVelocity = 15000; // max velocity in steps/s
        int setpointPosition; // in units of steps
};