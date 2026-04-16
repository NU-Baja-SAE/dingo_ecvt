#include <Arduino.h>
#include "DRV8462.h"
#include "encoder.h"
#include <string>
#include "pid.h"


class Motor {
    public:
        Motor();
        void init();
        void enable();
        void disable();
        int getPosition();
        void setPosition(int position);
        void setSetpoint(int position);
        std::string log();
        uint16_t getFault();
        void setHome(int homePosition);
        
    private:
        void startTimer();
        DRV8462 driver;
        Encoder encoder;
        void isrCallback();
        esp_timer_handle_t timer_handle;
        PID pid{1, 0, 0};

        int driver_step_count = 0; // track the total number of steps commanded to the driver


        int currentPosition; // in units of steps
        static const int maxAcceleration = 80000; // max acceleration in steps/s^2
        static const int maxVelocity = 15000; // max velocity in steps/s
        int setpointPosition; // in units of steps
        int lead_angle_log = 0;
};