// This fileimplements the control system for the ECVT

#include "controller.h"
#include <Arduino.h>
#include "pins.h"

Controller::Controller() : motor(), enginePulseCounter(), secondaryPulseCounter(), can(CAN_TX_PIN, CAN_RX_PIN) {}


/**
 * @brief Starts the control timer and the motor timer. The control timer will call the timerCallback method every CONTROLLER_TIMER_RATE milliseconds, and the motor timer will call its own timerCallback method every MOTOR_TIMER_RATE milliseconds.
 * 
 */
void Controller::startTimer()
{
    TimerHandle_t controller_timer = xTimerCreate("controller_timer",
                                             pdMS_TO_TICKS(CONTROLLER_TIMER_RATE),
                                             pdTRUE,
                                             (void *)this, // Pass the Controller instance as timer ID
                                             [](TimerHandle_t xTimer) {
                                                 // Timer callback function lambda
                                                 // retrieve the Controller instance from the timer ID and call the timerCallback method
                                                 Controller* controller = static_cast<Controller*>(pvTimerGetTimerID(xTimer));
                                                 controller->timerCallback();
                                             });

    if (!controller_timer)
    {
        Serial.printf("ERROR: Brake timer could not be created\n");
    }

    if (xTimerStart(controller_timer, 0) != pdPASS)
    {
        Serial.printf("ERROR: Brake timer could not be started\n");
    }

    motor.startTimer(); // Start the motor timer as well
}


/**
 * @brief Timer callback function for the control timer. This function will be called every CONTROLLER_TIMER_RATE milliseconds. It currently just prints a message to the console, but it will eventually contain the main control loop logic for the ECVT.
 * 
 */
void Controller::timerCallback()
{
    Serial.println("Hello from control timer callback!");
}