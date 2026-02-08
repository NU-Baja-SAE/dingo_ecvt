// This file implements motor control for the ECVT
#include "motor.h"

Motor::Motor() : currentPosition(0), targetPosition(0) {}


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
 * @brief Timer callback function for the motor timer. This function will be called every MOTOR_TIMER_RATE milliseconds. It currently just prints a message to the console, but it will eventually contain the logic for controlling the motor based on the current position and target position.
 * 
 */
void Motor::timerCallback()
{
    Serial.println("Hello from motor timer callback!");
}