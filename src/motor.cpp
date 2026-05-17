#include "motor.h"
#include "config.h"

Motor::Motor() : currentPosition(0), setpointPosition(0), currentVelocity(0.0f), stepAccumulator(0.0f), driver(), encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTER_ID) {}

void Motor::init()
{
    this->driver.begin();
    this->startTimer();
}

/**
 * @brief Start the FreeRTOS timer that drives motion updates.
 */
void Motor::startTimer()
{

    TimerHandle_t motor_timer = xTimerCreate("motor_timer",
                                             pdMS_TO_TICKS(MOTOR_TIMER_RATE),
                                             pdTRUE,
                                             (void *)this, // Pass the Motor instance as timer ID
                                             [](TimerHandle_t xTimer)
                                             {
                                                 // Timer callback function lambda
                                                 // retrieve the Motor instance from the timer ID and call the timerCallback method
                                                 Motor *motor = static_cast<Motor *>(pvTimerGetTimerID(xTimer));
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
 * @brief Set the target motor position in step units.
 * @param position Target step position (0 = idle).
 */
void Motor::setSetpoint(int position)
{
    this->setpointPosition = position;
}

int Motor::getPosition()
{
    return this->currentPosition;
}

int Motor::getSetpoint()
{
    return this->setpointPosition;
}

void Motor::setPosition(int position)
{
    this->currentPosition = position;
    this->encoder.setCount(position);
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
 * @brief Motor control tick that applies acceleration/velocity limits.
 */
void Motor::timerCallback()
{
    float timeStep = (float)MOTOR_TIMER_RATE / 1000.0f - 0.00005; // subtract 50us to ensure steps finish before next tick
    
    // Update current position from encoder feedback.
    this->currentPosition = this->encoder.getSteps();
    this->currentVelocity = (this->currentPosition - this->lastPosition) / timeStep; // calculate velocity based on change in position over time step

    // Calculate the ideal steps and speed.
    int stepsToMove = this->setpointPosition - this->currentPosition;
    int speed_hz = stepsToMove / timeStep; // speed proportional to the number of steps, with a maximum of maxVelocity

    // Determine which acceleration limit to use based on motion direction.
    int maxAcceleration = this->currentVelocity > 0 ? maxAcceleration_pos : maxAcceleration_neg;

    // Limit acceleration.
    float acceleration = (speed_hz - this->currentVelocity) / timeStep;
    if (acceleration > maxAcceleration)
    {
        speed_hz = this->currentVelocity + maxAcceleration * timeStep;
    }
    else if (acceleration < -maxAcceleration)
    {
        speed_hz = this->currentVelocity - maxAcceleration * timeStep;
    }

    // Limit speed.
    if (speed_hz > maxVelocity)
    {
        speed_hz = maxVelocity;
    }
    else if (speed_hz < -maxVelocity)
    {
        speed_hz = -maxVelocity;
    }

    // Set steps to move based on the limited speed.
    if (abs(stepsToMove) > abs(speed_hz * timeStep))
        if ((int)(speed_hz * timeStep) != 0)
            stepsToMove = speed_hz * timeStep;

    // Decelerate if we would overshoot the setpoint.
    float distanceToSetpoint = this->setpointPosition - this->currentPosition;

    

    float stoppingDistance = (speed_hz * speed_hz) / (2 * maxAcceleration);
    if (abs(distanceToSetpoint) < stoppingDistance)
    {        
        speed_hz = sqrt(2 * maxAcceleration * abs(distanceToSetpoint)) * (distanceToSetpoint > 0 ? 1 : -1);
        stepsToMove = speed_hz * timeStep;
    }

    // Prevent overshoot caused by step quantization.
    if (abs(stepsToMove) > abs(distanceToSetpoint))
    {        stepsToMove = distanceToSetpoint;
    }

    debugPrintf(">stepsToMove:%d\n", stepsToMove);
    this->driver.moveSteps(stepsToMove, abs(speed_hz));
    this->lastPosition = this->currentPosition; // update last position for velocity calculation in the next timer callback

}



std::string Motor::log()
{
    return "\n>pos:" + std::to_string(this->currentPosition) + "\n>vel:" + std::to_string(this->currentVelocity) + "\n>setpoint:" + std::to_string(this->setpointPosition);
}


uint16_t Motor::getFault()
{
    return this->driver.readFault();
}

void Motor::setHome(int homePosition) {
    this->currentPosition = homePosition;
    // this->setpointPosition = homePosition;
    this->currentVelocity = 0.0f;
    this->stepAccumulator = 0.0f;
    this->encoder.setCount(homePosition);
}
