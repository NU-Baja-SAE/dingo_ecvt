#include <Arduino.h>
#include <SPI.h>
#include "driver/rmt.h"
#include "soc/rmt_reg.h"

#include "DRV8462_REGMAP.h"


#define RMT_CHANNEL RMT_CHANNEL_0
#define MAX_PULSES 2000 // maximum number of pulses to send in one batch

/**
 * @brief DRV8462 stepper driver wrapper with SPI + RMT support.
 */
class DRV8462
{
public:
    DRV8462();
    ~DRV8462();

    /**
     * @brief Initialize SPI, pins, RMT, and driver configuration.
     */
    void begin();

    /**
     * @brief Enable or disable the power stage.
     */
    void enable();
    void disable();

    /**
     * @brief Move a fixed number of steps at a constant speed.
     * @param steps Step count (sign indicates direction).
     * @param speed_hz Step rate in Hz.
     */
    void moveSteps(int steps, int speed_hz);

    /**
     * @brief Move using a simple trapezoidal profile.
     */
    void moveTrapazoidal(int steps, int max_speed_hz, int acceleration_hz_per_sec);

    /**
     * @brief Stop RMT output.
     */
    void stop();

    /**
     * @brief Read fault and diagnostic registers.
     */
    uint16_t readFault();
    uint16_t readDiag2();
    bool isAtqLearningDone();
    void printAtqLearnedParameters();

    /**
     * @brief Handle driver fault conditions.
     */
    void faultDetected();

private:
    rmt_item32_t pulse_buf[MAX_PULSES];
    SPIClass *spi;
    bool atqLearningPending;
    bool atqLearningInProgress;
    bool atqLearningComplete;
    unsigned long atqLearningMotionStartMs;
    unsigned long atqLearningStartMs;
    void setupAutoTorque();
    void serviceAutoTorqueLearning(bool motorIsStepping);
    void spiWriteRegister(uint8_t address, uint16_t data);
    uint16_t spiReadRegister(uint8_t address);
    void setupRMT();
};
