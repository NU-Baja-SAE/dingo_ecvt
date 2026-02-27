// This file implements motor control for the ECVT
#include "motor.h"
#include "config.h"

Motor::Motor() : currentPosition(0), setpointPosition(0), currentVelocity(0.0f), stepAccumulator(0.0f), driver() {}


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

    TimerHandle_t motor_timer = xTimerCreate("motor_timer",
                                             pdMS_TO_TICKS(MOTOR_TIMER_RATE),
                                             pdTRUE,
                                             (void *)this, // Pass the Motor instance as timer ID
                                             [](TimerHandle_t xTimer) {
                                                 // Timer callback function lambda
                                                 // retrieve the Motor instance from the timer ID and call the timerCallback method
                                                 Motor* motor = static_cast<Motor*>(pvTimerGetTimerID(xTimer));
                                                 motor->timerCallback();
                                             });

    if (!motor_timer)
    {
        Serial.printf("ERROR: Motor timer could not be created\n");
    }

    if (xTimerStart(motor_timer, 0) != pdPASS)
    {
        Serial.printf("ERROR: Motor timer could not be started\n");
    }
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

void Motor::enable()
{
    this->driver.enable();
}

void Motor::disable()
{
    this->driver.disable();
}

/**
 * @brief Timer callback function for the motor timer. This function will be called every MOTOR_TIMER_RATE milliseconds. It calculates the number of steps needed to move from the current position to the setpoint position, and commands the driver to move that many steps at a speed proportional to the number of steps. It then updates the current position based on the steps commanded.
 * 
 */
void Motor::timerCallback()
{
    int steps = this->setpointPosition - this->currentPosition;
    
    float accel = (steps - this->currentVelocity * (MOTOR_TIMER_RATE / 1000.0f)) / (MOTOR_TIMER_RATE / 1000.0f); // Calculate the acceleration needed to reach the setpoint in the next time step
    if (accel > maxAcceleration)
    {
        accel = maxAcceleration;
    }
    else if (accel < -maxAcceleration)
    {
        accel = -maxAcceleration;
    }

    this->currentVelocity += accel * (MOTOR_TIMER_RATE / 1000.0f); // Update the current velocity based on the acceleration, ensuring we don't exceed max acceleration
    if (this->currentVelocity > this->maxVelocity) // Limit the maximum velocity to prevent commanding the driver to move too fast
    {
        this->currentVelocity = this->maxVelocity;
    }
    else if (this->currentVelocity < -this->maxVelocity)
    {
        this->currentVelocity = -this->maxVelocity;
    }

    this->stepAccumulator += this->currentVelocity * (MOTOR_TIMER_RATE / 1000.0f); // Accumulate fractional steps to allow proper low-speed operation
    int stepsToMove = (int)this->stepAccumulator; // Extract whole steps, keeping fractional remainder
    this->stepAccumulator -= stepsToMove;

    this->driver.moveSteps(stepsToMove, abs(this->currentVelocity)); // Move forward 
    this->currentPosition += stepsToMove; // Update the current position based on the steps commanded

    Serial.printf(">Setpoint:%d\n>Current_Position:%d\n>Steps_to_Move:%d\n>Current_Velocity:%.2f\n", this->setpointPosition, this->currentPosition, stepsToMove, this->currentVelocity);
     

}

