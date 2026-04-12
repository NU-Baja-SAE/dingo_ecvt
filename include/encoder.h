#include "driver/pcnt.h"
#include <Arduino.h>


class Encoder {
public:
    Encoder(gpio_num_t a, gpio_num_t b, pcnt_unit_t counterId);
    int getCount();
    int getSteps();
    void resetCount();
private:
    pcnt_unit_t counterId;

};