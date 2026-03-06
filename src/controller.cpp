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
    controller_timer = xTimerCreate("controller_timer",
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
        Serial.printf("ERROR: Controller timer could not be created\n");
    }

    if (xTimerStart(controller_timer, 0) != pdPASS)
    {
        Serial.printf("ERROR: Controller timer could not be started\n");
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
    
    float engineRPM = enginePulseCounter.getRPM();
    float secondaryRPM = secondaryPulseCounter.getRPM();
    float gearRatio = 0;
    
    switch (this->controlMode)
    {
        case POWER:
        /* code */
            gearRatio = this->powerGearRatio(engineRPM, secondaryRPM);
            motorSetpoint = this->gearRatioToSetpoint(gearRatio);
        break;
    
    default:
        break;
    }

    motorSetpoint = sin(2 * PI * 0.5 * millis() / 1000.0) * 1600; // Example: Sine wave setpoint for testing (amplitude of 750 steps, frequency of 0.5 Hz)
    
    // step trajectory generation: 300 step square wave with a period of 4 seconds (2 seconds at +300 steps, 2 seconds at -300 steps)
    motorSetpoint = (millis() / 2000) % 2 == 0 ? 4 * 3200 : 0;

    // set motor setpoint
    motor.setSetpoint(motorSetpoint);
    // Serial.printf(">Motorsetpoint_ctrl:%d\n", motorSetpoint);

    // send motor setpoint and test sine-wave telemetry over CAN bus
    CanMessage msg(0x101, motorSetpoint); // sends current motor setpoint (example CAN ID, change as needed)
    can.writeMessage(msg, 0); // Non-blocking write

    float val = (float) (sin(millis() / 1000.0) * 1000);
    CanMessage msg2(0x100, val); // sends synthetic sine-wave telemetry value (example CAN ID, change as needed)
    Serial.printf(">CAN_SIN:%f\n", val);
    can.writeMessage(msg2, 0); // Non-blocking write
    
}



float Controller::powerGearRatio(float engineRPM, float secondaryRPM)
{
    if (secondaryRPM < SLIP_SPEED) { // belt is slipping, should either be in idle, or lerp between idle and low based on engine RPM
        if (engineRPM < ENGINE_ENGAGE_RPM ) {
            return 100; // idle == high gear ratio
        } else {
            float t = (engineRPM - ENGINE_ENGAGE_RPM) / (ENGINE_IDEAL_RPM - ENGINE_ENGAGE_RPM);
            return 100 + t * (LOW_GEAR - 100); // lerp between idle and low gear ratio
        }

    } else if (secondaryRPM < CRUISE_LOW) { // belt is not slipping, but engine RPM is still below ideal
        return LOW_GEAR; 

    } else  if (secondaryRPM < CRUISE_HIGH) { // car is in main range, should lerp between low and high gear based on secondary rpm
        float t = (secondaryRPM - CRUISE_LOW) / (CRUISE_HIGH - CRUISE_LOW);
        return LOW_GEAR + t * (HIGH_GEAR - LOW_GEAR);

    } else { // car is going fast, should be in highest gear
        return HIGH_GEAR;
    }
}

int Controller::gearRatioToSetpoint(int gearRatio)
{
    // This function should convert the desired gear ratio to a motor setpoint in units of steps. The exact conversion will depend on the specifics of the ECVT design, such as the relationship between motor position and gear ratio. For now, we will just return a placeholder value.
    return 0; // TODO: implement this function based on the ECVT design
}