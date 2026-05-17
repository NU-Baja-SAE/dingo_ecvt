#include "pulse_counter.h"
#include <limits.h>
#include "config.h"


/**
 * @brief Construct a pulse counter configured for a Hall sensor.
 * @param hallPin GPIO pin connected to the Hall sensor output.
 * @param counterId PCNT unit to use (PCNT_UNIT_0, PCNT_UNIT_1, etc.).
 * @param magnetCount Number of magnets on the wheel.
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

    // Set filter to ignore glitches. Units are APB clock cycles.
    pcnt_set_filter_value(counterId, 1000);
    pcnt_filter_enable(counterId);

    // Clear and start the counter.
    pcnt_counter_clear(counterId);
    pcnt_counter_resume(counterId);
}

/**
 * @brief Get the current count of the pulse counter.
 */
int PulseCounter::getCount() {
    int16_t pulse_count;
    pcnt_get_counter_value(counterId, &pulse_count);
    return pulse_count;
}

/**
 * @brief Reset the pulse counter to zero.
 */
void PulseCounter::resetCount() {
    pcnt_counter_clear(counterId);
    hasLastSample = false;
    lastCount = 0;
    lastSampleTimeMs = 0;
}


/**
 * @brief Calculate RPM from pulse delta and elapsed time.
 * @return RPM value.
 */
float PulseCounter::getRPM() {
    if (magnetCount <= 0) {
        return 0.0f;
    }

    int16_t currentCount;
    pcnt_get_counter_value(counterId, &currentCount);

    uint32_t currentTimeMs = millis();

    if (!hasLastSample) {
        lastCount = currentCount;
        lastSampleTimeMs = currentTimeMs;
        hasLastSample = true;
        return 0.0f;
    }

    uint32_t elapsedMs = currentTimeMs - lastSampleTimeMs;
    if (elapsedMs == 0) {
        return 0.0f;
    }

    int32_t deltaCount = static_cast<int32_t>(currentCount) - static_cast<int32_t>(lastCount);

    // Correct for int16 counter wrap between samples.
    if (deltaCount < INT16_MIN) {
        deltaCount += (INT16_MAX - INT16_MIN + 1);
    } else if (deltaCount > INT16_MAX) {
        deltaCount -= (INT16_MAX - INT16_MIN + 1);
    }

    float pulsesPerRevolution = static_cast<float>(magnetCount) * EDGES_PER_MAGNET;

    float rpm = 0.0f;
    if (deltaCount > 0 && pulsesPerRevolution > 0.0f) {
        rpm = (static_cast<float>(deltaCount) * 60000.0f) / (pulsesPerRevolution * static_cast<float>(elapsedMs));
    }

    lastCount = currentCount;
    lastSampleTimeMs = currentTimeMs;

    // Apply low-pass filter to smooth out RPM readings.
    rpm = rpmFilter.filter(rpm);
    filteredRPM = rpm;

    return rpm;
}
