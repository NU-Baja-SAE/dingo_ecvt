#ifndef FILTER_H
#define FILTER_H

/**
 * @brief Simple single-pole low-pass filter for sensor smoothing.
 */
class LowPassFilter {
public:
    /**
     * @brief Construct a low-pass filter.
     * @param alpha Smoothing factor in [0,1], higher favors new values.
     */
    LowPassFilter(float alpha) : alpha(alpha), hasLastValue(false), lastValue(0.0f) {}    

    /**
     * @brief Apply the filter to a new sample.
     * @param newValue New sample value.
     * @return Filtered output value.
     */
    float filter(float newValue) {
        if (!hasLastValue) {
            lastValue = newValue;
            hasLastValue = true;
            return newValue;
        }
        lastValue = alpha * newValue + (1 - alpha) * lastValue;
        return lastValue;
    }

private:
    float alpha; // smoothing factor between 0 and 1
    bool hasLastValue;
    float lastValue;
};

#endif // FILTER_H
