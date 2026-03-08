#include "driver/pcnt.h"
#include <Arduino.h>
#include "filter.h"

class PulseCounter {
public:
    PulseCounter(gpio_num_t hallPin, pcnt_unit_t counterId, int magnetCount);
    int getCount();
    float getRPM();
    void resetCount();

private:
    pcnt_unit_t counterId;
    int magnetCount; // number of magnets on the wheel, used for RPM calculation
    int16_t lastCount = 0;
    uint32_t lastSampleTimeMs = 0;
    bool hasLastSample = false;
    LowPassFilter rpmFilter = LowPassFilter(0.2); // low-pass filter with alpha = 0.1 for smoothing RPM readings
};
