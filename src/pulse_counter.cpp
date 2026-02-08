#include "pulse_counter.h"
#include <limits.h>


/**
 * @brief Construct a new Pulse Counter:: Pulse Counter object
 * 
 * @param hallPin GPIO pin connected to the hall sensor output
 * @param counterId PCNT unit to use for this counter (e.g. PCNT_UNIT_0, PCNT_UNIT_1, etc.)
 * @param magnetCount Number of magnets on the wheel, used for RPM calculation
 */
PulseCounter::PulseCounter(gpio_num_t hallPin, pcnt_unit_t counterId, int magnetCount) : counterId(counterId), magnetCount(magnetCount) {

    pcnt_config_t config = {};

    config.unit = counterId;
    config.channel = PCNT_CHANNEL_0; // only use channel 0 of each counter unit.

    // Set signal and control input GPIOs
    config.pulse_gpio_num = hallPin;
    config.ctrl_gpio_num = PCNT_PIN_NOT_USED;

    // Set counting modes
    config.pos_mode = PCNT_COUNT_INC; // count on rising edge
    config.neg_mode = PCNT_COUNT_INC; // count on falling edge
    config.lctrl_mode = PCNT_MODE_KEEP;
    config.hctrl_mode = PCNT_MODE_KEEP;

    // Set counter limits (required)
    config.counter_h_lim = INT16_MAX;
    config.counter_l_lim = INT16_MIN;

    pcnt_unit_config(&config);

    // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(counterId, 1000);
    pcnt_filter_enable(counterId);

    // clear and start the counter
    pcnt_counter_clear(counterId);
    pcnt_counter_resume(counterId);
}


/**
 * @brief Gets the current count of the pulse counter
 * 
 */
int PulseCounter::getCount() {
    int16_t pulse_count;
    pcnt_get_counter_value(counterId, &pulse_count);
    return pulse_count;
}

/**
 * @brief Resets the pulse counter to zero
 * 
 */
void PulseCounter::resetCount() {
    pcnt_counter_clear(counterId);
}


/**
 * @brief Calculates the RPM based on the current count and magnet count
 * 
 * @return float RPM value
 */
float PulseCounter::getRPM() {
   return 0; // TODO: implement RPM calculation

}