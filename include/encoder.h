#include "driver/pcnt.h"
#include <Arduino.h>


/**
 * @brief Quadrature encoder reader using ESP32 PCNT hardware.
 */
class Encoder {
public:
    /**
     * @brief Construct a quadrature encoder reader.
     * @param a Encoder channel A GPIO.
     * @param b Encoder channel B GPIO.
     * @param counterId PCNT unit to use.
     */
    Encoder(gpio_num_t a, gpio_num_t b, pcnt_unit_t counterId);

    /**
     * @brief Read the accumulated count since last reset.
     */
    int getCount();

    /**
     * @brief Convert encoder counts to stepper steps.
     */
    int getSteps();

    /**
     * @brief Reset the count and stored offsets.
     */
    void resetCount();

    /**
     * @brief Set the logical count to a specific value.
     */
    void setCount(int count);

private:
    static constexpr int32_t COUNT_PER_REV = 4096;
    pcnt_unit_t counterId;
    int32_t offset = 0;
    int32_t full_revs = 0;
    int32_t last_count = 0;
};
