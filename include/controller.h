#include "motor.h"
#include "pulse_counter.h"
#include "BajaCan.h"
#include <string>
#include "filter.h"
#include "config.h"


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
            return motor.log() + "\n>Engine_RPM:" + std::to_string(enginePulseCounter.getFilteredRPM()) + "\n>brake_state:" + std::to_string(analogRead(BRAKE_PIN) > 1000 ? 1 : 0) + "\n>manual_mode:" + std::to_string(this->brake_pressed ? 1 : 0);
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
        bool brake_pressed = false; 
        Motor motor;
        PulseCounter enginePulseCounter;
        BajaCan can;
        ControlMode controlMode = HOMING; 
        // PID gearRatioPID = PID(20.0, 0.0, 0.0);

        float last_speed = 0.0f;

};


