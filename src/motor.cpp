// This file implements motor control for the ECVT
#include "motor.h"
#include "config.h"

Motor::Motor() : currentPosition(0), setpointPosition(0), driver(), encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTER_ID) {}

void Motor::init()
{
    this->driver.begin();
    this->startTimer();
}

/**
 * @brief Starts the motor timer. The motor timer will call the timerCallback method every MOTOR_TIMER_RATE milliseconds. This method should be called in the Controller::startTimer() method to ensure that the motor timer is started when the controller timer is started.
 *
 */
void Motor::startTimer()
{


    // start the 20khz timer
    const esp_timer_create_args_t timer_args = {
        .callback = [](void *arg) { // Timer callback function lambda
            Motor *motor = static_cast<Motor *>(arg);
            motor->isrCallback();
        },
        .arg = this, // Pass 'this' instance
        .name = "isr_timer"};
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, 50); // 50us period for 20kHz frequency
}

/**
 * @brief Sets the target position for the motor
 *
 * @param position in units of encoder counts, where 0 is the idle position, and the positive direction is towards the engine
 */
void Motor::setSetpoint(int position)
{
    this->setpointPosition = position;
}

int Motor::getPosition()
{
    return this->currentPosition;
}

void Motor::setPosition(int position)
{
    this->currentPosition = position;
    this->encoder.setCount(position);
    this->driver_step_count = position; // assume driver is in sync with encoder when manually setting position

}

void Motor::enable()
{
    this->driver.enable();
}

void Motor::disable()
{
    this->driver.disable();
}


std::string Motor::log()
{
    return ">pos:" + std::to_string(this->currentPosition) + "\n>driver_pos:" + std::to_string(this->driver_step_count) + "\n>setpoint:" + std::to_string(this->setpointPosition) + "\n>lead_angle:" + std::to_string(this->lead_angle_log);
}

uint16_t Motor::getFault()
{
    return this->driver.readFault();
}

void Motor::setHome(int homePosition)
{
    this->currentPosition = homePosition;
    this->setpointPosition = homePosition;
    this->encoder.setCount(homePosition);
}

void Motor::isrCallback()
{
    // 1. Pull the step pin low immediately if it was high from the last interrupt
    this->driver.pullStepLow(); 

    // 2. Read physical rotor position
    this->currentPosition = this->encoder.getSteps();

    // 3. Compute PID effort
    float effort = this->pid.compute(this->setpointPosition, this->currentPosition, 0.00005); 

    // 4. Calculate Lead Angle (Clamped to 1 full step / 16 microsteps)
    int lead_angle = clamp((int)effort, -16, 16); 

    // 5. Calculate where the Stator (magnetic field) needs to be
    long target_driver_pos = this->currentPosition + lead_angle; 

    // 6. Conditionally step the driver to catch up to the target
    if (this->driver_step_count < target_driver_pos) {
        this->driver.step(true); // Step Forward
        this->driver_step_count++;
    } 
    else if (this->driver_step_count > target_driver_pos) {
        this->driver.step(false); // Step Reverse
        this->driver_step_count--;
    }
    // If driver_step_count == target_driver_pos, do nothing! The motor holds.
    this->lead_angle_log = this->currentPosition - this->driver_step_count; // for logging and debugging
}