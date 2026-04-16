
#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), prev_error(0), integral(0) {}

    float compute(float setpoint, float measured_value, float dt) {
        float error = setpoint - measured_value;
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
private:
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
};

#endif // PID_H