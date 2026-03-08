// provide a simple low-pass filter for smoothing out sensor readings
#ifndef FILTER_H
#define FILTER_H

class LowPassFilter {
public:
    LowPassFilter(float alpha) : alpha(alpha), hasLastValue(false), lastValue(0.0f) {}    

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