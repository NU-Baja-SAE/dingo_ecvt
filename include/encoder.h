#include "driver/pcnt.h"
#include <Arduino.h>


class Encoder {
public:
    Encoder(gpio_num_t a, gpio_num_t b, pcnt_unit_t counterId);
    int getCount();
    int getSteps();
    void resetCount();
    void setCount(int count);
private:
    static constexpr int32_t COUNT_PER_REV = 4096;
    pcnt_unit_t counterId;
    int32_t offset = 0;
    int32_t full_revs = 0;
    int32_t last_count = 0;

};