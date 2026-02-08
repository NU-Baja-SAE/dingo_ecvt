#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"

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
    private:
        void timerCallback();
        Motor motor;
        PulseCounter enginePulseCounter;
        PulseCounter secondaryPulseCounter;
        BajaCan can;
        ControlMode controlMode = POWER; // default to power mode
};