#include "driver/pcnt.h"
#include <Arduino.h>
#include "filter.h"

/**
 * @brief Hall-effect pulse counter with RPM calculation and filtering.
 */
class PulseCounter {
public:
    /**
     * @brief Construct a pulse counter for a Hall sensor input.
     * @param hallPin GPIO pin connected to the Hall sensor.
     * @param counterId PCNT unit to use.
     * @param magnetCount Number of magnets per revolution.
     */
    PulseCounter(gpio_num_t hallPin, pcnt_unit_t counterId, int magnetCount);

    /**
     * @brief Read the raw pulse count from the PCNT unit.
     */
    int getCount();

    /**
     * @brief Compute RPM based on pulse delta and sample time.
     */
    float getRPM();

    /**
     * @brief Reset the PCNT unit and RPM state.
     */
    void resetCount();

    /**
     * @brief Return the filtered RPM value.
     */
    float getFilteredRPM() const {
        return filteredRPM;
    }

private:
    pcnt_unit_t counterId;
    int magnetCount; // number of magnets on the wheel, used for RPM calculation
    int16_t lastCount = 0;
    uint32_t lastSampleTimeMs = 0;
    bool hasLastSample = false;
    LowPassFilter rpmFilter = LowPassFilter(0.2);
    float filteredRPM = 0.0f;
};
