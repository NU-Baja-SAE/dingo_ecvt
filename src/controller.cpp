// This fileimplements the control system for the ECVT

#include "controller.h"
#include <Arduino.h>
#include "config.h"
#include "CanDatabase.h"

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
    int32_t motorSetpoint = 0;
    
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

    // set motor setpoint
    motor.setSetpoint(motorSetpoint);

    // send engine RPM and secondary RPM over CAN bus for telemetry
    CanMessage engineRpmMsg(CanDatabase::ENGINE_RPM.id, engineRPM); 
    can.writeMessage(engineRpmMsg, 0);

    CanMessage secondaryRpmMsg(CanDatabase::SECONDARY_RPM.id, secondaryRPM); 
    can.writeMessage(secondaryRpmMsg, 0);

    CanMessage motorSetpointMsg(CanDatabase::MOTOR_SETPOINT.id, motorSetpoint);
    can.writeMessage(motorSetpointMsg, 0);

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


// PID controler to convert desired gear ratio to motor setpoint
int Controller::gearRatioToSetpoint(float gearRatio)
{
    if (gearRatio < HIGH_GEAR) {
        return MAX_MOTOR_SETPOINT;
    } else if (gearRatio > 50) {
        return MIN_MOTOR_SETPOINT; 
    } 
    
    int setpoint = this->gearRatioPID.calculate(gearRatio, enginePulseCounter.getRPM() / secondaryPulseCounter.getRPM(), CONTROLLER_TIMER_RATE / 1000.0);
    return constrain(setpoint, MIN_MOTOR_SETPOINT, MAX_MOTOR_SETPOINT);
}