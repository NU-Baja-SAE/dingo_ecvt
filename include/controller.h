#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"

#define CONTROLLER_TIMER_RATE 1000 // ms

class Controller {
    public:
        Controller();
        void startTimer();
    private:
        void timerCallback();
        Motor motor;
        PulseCounter enginePulseCounter;
        PulseCounter secondaryPulseCounter;
        BajaCan can;
};