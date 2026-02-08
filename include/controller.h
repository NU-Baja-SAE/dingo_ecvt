#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"


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
};