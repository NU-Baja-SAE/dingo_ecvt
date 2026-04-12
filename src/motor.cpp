// This file implements motor control for the ECVT
#include "motor.h"
#include "config.h"

Motor::Motor() : currentPosition(0), setpointPosition(0), currentVelocity(0.0f), stepAccumulator(0.0f), driver(), encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_COUNTER_ID) {}

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
    float timeStep = (float)MOTOR_TIMER_RATE / 1000.0f - 0.00005; // time step in seconds, subract 50us to ensure steps finish before next timer callback
    
    // update current position from encoder
    this->currentPosition = this->encoder.getSteps();


    // calculate the ideal steps and speed
    int stepsToMove = this->setpointPosition - this->currentPosition;
    int speed_hz = stepsToMove / timeStep; // speed proportional to the number of steps, with a maximum of maxVelocity

    

    // limit acceleration
    float acceleration = (speed_hz - this->currentVelocity) / timeStep;
    if (acceleration > maxAcceleration)
    {
        speed_hz = this->currentVelocity + maxAcceleration * timeStep;
    }
    else if (acceleration < -maxAcceleration)
    {
        speed_hz = this->currentVelocity - maxAcceleration * timeStep;
    }

    // limit speed
    if (speed_hz > maxVelocity)
    {
        speed_hz = maxVelocity;
    }
    else if (speed_hz < -maxVelocity)
    {
        speed_hz = -maxVelocity;
    }

    // set steps to move based on the limited speed
    if (abs(stepsToMove) > abs(speed_hz * timeStep))
        if ((int)(speed_hz * timeStep) != 0)
            stepsToMove = speed_hz * timeStep;

    // implement deceleration by checking if our future position and velocity would cause us to overshoot the setpoint, and if so, limit the steps to move to ensure we stop at the setpoint
    float distanceToSetpoint = this->setpointPosition - this->currentPosition;
    float stoppingDistance = (speed_hz * speed_hz) / (2 * maxAcceleration);
    if (abs(distanceToSetpoint) < stoppingDistance)
    {        
        speed_hz = sqrt(2 * maxAcceleration * abs(distanceToSetpoint)) * (distanceToSetpoint > 0 ? 1 : -1);
        stepsToMove = speed_hz * timeStep;
    }

    // prevent overshooting due to step quantization by checking if the steps to move would cause us to overshoot the setpoint, and if so, limit the steps to move to ensure we do not overshoot
    if (abs(stepsToMove) > abs(distanceToSetpoint))
    {        stepsToMove = distanceToSetpoint;
    }

    debugPrintf(">stepsToMove:%d\n", stepsToMove);
    this->driver.moveSteps(stepsToMove, abs(speed_hz));
    // For simplicity, we will assume that the motor moves the commanded steps instantly. In reality, you would want to track the actual position using encoder feedback and update currentPosition accordingly.
    // this->currentPosition += stepsToMove;
    this->currentVelocity = speed_hz;

}



std::string Motor::log()
{
    return ">pos:" + std::to_string(this->currentPosition) + "\n>vel:" + std::to_string(this->currentVelocity) + "\n>setpoint:" + std::to_string(this->setpointPosition);   
}


uint16_t Motor::getFault()
{
    return this->driver.readFault();
}

void Motor::setHome(int homePosition) {
    this->currentPosition = homePosition;
    this->setpointPosition = homePosition;
    this->currentVelocity = 0.0f;
    this->stepAccumulator = 0.0f;
}