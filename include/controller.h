#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"
#include <string>
#include "filter.h"


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


enum ControlMode {
    TORQUE,
    POWER,
    MANUAL,
    BRAKE,
    DEBUG,
    HOMING
};


class Controller {
    public:
        Controller();
        void init();
        TimerHandle_t controller_timer; // Made public for health checks in main.cpp
        std::string log() {
            // return motor.log() + "\n>Engine_RPM:" + std::to_string(enginePulseCounter.getFilteredRPM()) + 
            //        "\n>Secondary_RPM:" + std::to_string(secondaryPulseCounter.getFilteredRPM()) + "\n>lin_voltage:" + std::to_string(analogRead(GPIO_NUM_39));
            return motor.log() + "\n>Engine_RPM:" + std::to_string(enginePulseCounter.getFilteredRPM()) + 
                   "\n>Secondary_RPM:" + std::to_string(secondaryPulseCounter.getFilteredRPM());
        }

    private:
        void timerCallback();
        // float powerGearRatio(float engineRPM, float secondaryRPM);
        // int gearRatioToSetpoint(float gearRatio);
        int rpmToSetpoint(float engineRPM);
        int homingRoutine();
        void sendCan();
        void readCan();


        float last_Error;
        float brake_pos = 0.0f;
        Motor motor;
        PulseCounter enginePulseCounter;
        PulseCounter secondaryPulseCounter;
        BajaCan can;
        ControlMode controlMode = HOMING; 
        // PID gearRatioPID = PID(20.0, 0.0, 0.0);

};


