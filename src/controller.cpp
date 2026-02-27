// This fileimplements the control system for the ECVT

#include "controller.h"
#include <Arduino.h>
#include "config.h"

Controller::Controller() : motor(),
                           enginePulseCounter(PRIMARY_HALL_PIN, PRIMARY_COUNTER_ID, PRIMARY_MAGNET_COUNT), 
                           secondaryPulseCounter(SECONDARY_HALL_PIN, SECONDARY_COUNTER_ID, SECONDARY_MAGNET_COUNT), 
                           can(CAN_TX_PIN, CAN_RX_PIN)
{
}

/**
 * @brief Starts the control timer and the motor timer and starts can. The control timer will call the timerCallback method every CONTROLLER_TIMER_RATE milliseconds, and the motor timer will call its own timerCallback method every MOTOR_TIMER_RATE milliseconds.
 *
 */
void Controller::init()
{
    TimerHandle_t controller_timer = xTimerCreate("controller_timer",
                                                  pdMS_TO_TICKS(CONTROLLER_TIMER_RATE),
                                                  pdTRUE,
                                                  (void *)this, // Pass the Controller instance as timer ID
                                                  [](TimerHandle_t xTimer)
                                                  {
                                                      // Timer callback function lambda
                                                      // retrieve the Controller instance from the timer ID and call the timerCallback method
                                                      Controller *controller = static_cast<Controller *>(pvTimerGetTimerID(xTimer));
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

    motor.init(); // Start the motor timer as well
    motor.enable(); // Enable the motor driver
    can.begin();        // Start the CAN bus
}

/**
 * @brief Timer callback function for the control timer. This function will be called every CONTROLLER_TIMER_RATE milliseconds. It currently just prints a message to the console, but it will eventually contain the main control loop logic for the ECVT.
 *
 */
void Controller::timerCallback()
{
    // Determine motor setpoint based on mode
    int motorSetpoint = 0;
    
    switch (this->controlMode)
    {
        case POWER:
        /* code */
        break;
    
    default:
        break;
    }

    motorSetpoint = sin(2 * PI * 0.5 * millis() / 1000.0) * 750; // Example: Sine wave setpoint for testing (amplitude of 750 steps, frequency of 0.5 Hz)
    
    // set motor setpoint
    motor.setSetpoint(motorSetpoint);
    
    CanMessage receivedMessage(0, 0);                    // Create an empty CanMessage object to store the received message
    esp_err_t ret = can.readMessage(receivedMessage, 0); // Non-blocking read
    if (ret == ESP_OK)
    {
        Serial.printf("Received CAN message with ID: 0x%X\n", receivedMessage.getFrame().identifier);
        // Process the received CAN message here
    }
    else if (ret != ESP_ERR_TIMEOUT)
    {
        Serial.printf("Error reading CAN message: %s\n", esp_err_to_name(ret));
    }
    else
    {
        Serial.println("No CAN message received (timeout)");
    }

    int engineCount = enginePulseCounter.getCount();
    int secondaryCount = secondaryPulseCounter.getCount();
    Serial.printf("Engine count: %d, Secondary count: %d\n", engineCount, secondaryCount);
}