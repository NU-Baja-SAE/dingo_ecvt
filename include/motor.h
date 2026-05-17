#include <Arduino.h>
#include "DRV8462.h"
#include "encoder.h"
#include <string>


/**
 * @brief Stepper motor wrapper for the DRV8462 driver and encoder feedback.
 */
class Motor {
    public:
        Motor();

        /**
         * @brief Initialize the driver and start the motor timer.
         */
        void init();

        /**
         * @brief Enable or disable the motor driver outputs.
         */
        void enable();
        void disable();

        /**
         * @brief Read or set motor state in step units.
         */
        int getPosition();
        int getSetpoint();
        void setPosition(int position);
        void setSetpoint(int position);

        /**
         * @brief Create a log snapshot for serial telemetry.
         */
        std::string log();

        /**
         * @brief Read the driver fault register.
         */
        uint16_t getFault();

        /**
         * @brief Reset the home position to the provided step offset.
         */
        void setHome(int homePosition);
        
    private:
        void startTimer();
        void timerCallback();

        DRV8462 driver;
        Encoder encoder;

        int currentPosition; // in units of steps
        int lastPosition; // in units of steps, used to calculate velocity
        float currentVelocity; // in units of steps/s
        float stepAccumulator; // accumulates fractional steps for sub-step precision
        static const int maxAcceleration_pos = 30000; // max acceleration in steps/s^2
        static const int maxAcceleration_neg = 120000; // max acceleration in steps/s^2
        static const int maxVelocity = 80000; // max velocity in steps/s
        int setpointPosition; // in units of steps
};
