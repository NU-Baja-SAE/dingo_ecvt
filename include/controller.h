#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"
#include <string>



enum ControlMode {
    TORQUE,
    POWER,
    MANUAL,
    BRAKE,
};


class Controller {
    public:
        Controller();
        void init();
        TimerHandle_t controller_timer; // Made public for health checks in main.cpp
        std::string log() {
            return motor.log();
        }

    private:
        void timerCallback();
        float powerGearRatio(float engineRPM, float secondaryRPM);
        int gearRatioToSetpoint(float gearRatio);
        Motor motor;
        PulseCounter enginePulseCounter;
        PulseCounter secondaryPulseCounter;
        BajaCan can;
        ControlMode controlMode = POWER; // default to power mode
        PID gearRatioPID = PID(20.0, 0.0, 0.0); // PID controller with example gains
};


class PID {
    public:
        PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}
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