#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"
#include <string>



enum ControlMode {
    TORQUE,
    POWER,
    MANUAL,
    BRAKE,
};


class Controller {
    public:
        Controller();
        void init();
        TimerHandle_t controller_timer; // Made public for health checks in main.cpp
        std::string log() {
            return motor.log();
        }

    private:
        void timerCallback();
        float powerGearRatio(float engineRPM, float secondaryRPM);
        int gearRatioToSetpoint(int gearRatio);
        Motor motor;
        PulseCounter enginePulseCounter;
        PulseCounter secondaryPulseCounter;
        BajaCan can;
        ControlMode controlMode = POWER; // default to power mode
};