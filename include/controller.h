#ifndef CONTROLLER_H
#define CONTROLLER_H


#include "motor.h"
#include "pulse_counter.h"
#include <string>
#include "filter.h"
#include "config.h"


/**
 * @brief Minimal PID helper used for local control experiments.
 */
class PID {
    public:
        /**
         * @brief Construct a PID with configured gains.
         * @param kp Proportional gain.
         * @param ki Integral gain.
         * @param kd Derivative gain.
         */
        PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

        /**
         * @brief Compute PID output using the current error and timestep.
         * @param setpoint Target value.
         * @param measured Current value.
         * @param dt Step time in seconds.
         * @return PID output.
         */
        float calculate(float setpoint, float measured, float dt) {
            float error = setpoint - measured;
            integral += error * dt;
            float derivative = (error - prevError) / dt;
            prevError = error;
            return kp * error + ki * integral + kd * derivative;
        }

    private:
        float kp;
        float ki;
        float kd;
        float prevError;
        float integral;
};


/**
 * @brief High-level control modes for the CVT controller.
 */
enum ControlMode {
    TORQUE,
    POWER,
    MANUAL,
    BRAKE,
    DEBUG,
    HOMING,
    BRAKE_CHECK,
    ACCELERATION
};

/**
 * @brief Convert a control mode enum to a log-friendly string.
 * @param mode Mode to convert.
 * @return String representation of the mode.
 */
inline std::string controlModeToString(ControlMode mode) {
    switch (mode) {
        case TORQUE: return "TORQUE";
        case POWER: return "POWER";
        case MANUAL: return "MANUAL";
        case BRAKE: return "BRAKE";
        case DEBUG: return "DEBUG";
        case HOMING: return "HOMING";
        case BRAKE_CHECK: return "BRAKE_CHECK";
        case ACCELERATION: return "ACCELERATION";
        default: return "UNKNOWN";
    }
}


/**
 * @brief Coordinates CVT control, motor motion, and telemetry.
 */
class Controller {
    public:
        /**
         * @brief Construct the controller and its subsystems.
         */
        Controller();

        /**
         * @brief Initialize timers, I/O, motor driver, and telemetry links.
         */
        void init();

        /**
         * @brief FreeRTOS timer handle used by main health checks.
         */
        TimerHandle_t controller_timer;

        /**
         * @brief Send one telemetry sample over the enabled links (ESP-NOW, SD logger).
         */
        void sendTelemetry();

        /**
         * @brief Build a human-readable telemetry snapshot.
         * @return Multi-line log string.
         */
        std::string log() {
            return motor.log() +
                   "\n>Engine_RPM:" + std::to_string(enginePulseCounter.getFilteredRPM()) +
                   "\n>brake_state:" + std::to_string(digitalRead(BRAKE_PIN) == HIGH ? 1 : 0) +
                   "\n>manual_mode:" + std::to_string(this->brake_pressed ? 1 : 0) +
                   "\n>control_mode:" + controlModeToString(this->controlMode) + "|t";
        }

    private:
        /**
         * @brief Periodic control tick executed by the controller timer.
         */
        void timerCallback();

        /**
         * @brief Translate engine RPM into a sheave position setpoint.
         * @param engineRPM Current engine speed.
         * @return Motor setpoint in steps.
         */
        int rpmToSetpoint(float engineRPM);

        /**
         * @brief Perform the limit-switch homing sequence.
         * @return Motor setpoint for the current homing step.
         */
        int homingRoutine();

        /**
         * @brief Reset homing state to begin a new sequence.
         */
        void resetHomingRoutine();

        /**
         * @brief Update control mode based on manual mode input.
         */
        void setMode();

        /**
         * @brief Bring up WiFi in station mode and register the ESP-NOW peer.
         */
        void initEspNow();

        bool espNowReady = false;

        float last_Error;
        bool brake_pressed = false;
        bool homingFirstTrigger = false;
        unsigned long homingTriggerTime = 0;
        Motor motor;
        PulseCounter enginePulseCounter;
        ControlMode controlMode = HOMING;
        float last_speed = 0.0f;
};




#endif // CONTROLLER_H
