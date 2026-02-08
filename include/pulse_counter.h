#include "driver/pcnt.h"
#include <Arduino.h>

class PulseCounter {
public:
    PulseCounter(gpio_num_t hallPin, pcnt_unit_t counterId, int magnetCount);
    int getCount();
    float getRPM();
    void resetCount();

private:
    pcnt_unit_t counterId;
    int magnetCount; // number of magnets on the wheel, used for RPM calculation

};