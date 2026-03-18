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
    this->last_Error = 0.0f;
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

    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);

    motor.init();   // Start the motor timer as well
    motor.enable(); // Enable the motor driver
    can.begin();    // Start the CAN bus
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
    bool limitSwitchState = digitalRead(LIMIT_SWITCH_PIN) == LOW;

    switch (this->controlMode)
    {
    case POWER:
        /* code */
        motorSetpoint = this->rpmToSetpoint(engineRPM);

        if (limitSwitchState)
        {
            this->motor.setHome(LIMIT_SWITCH_HOME_OFFSET);
            motorSetpoint = 0; 
        }
        break;
    case HOMING:
        motorSetpoint = this->homingRoutine();
        break;
    case DEBUG:

        motorSetpoint = constrain(map(millis(), 0, 1000, 0, STEPS_PER_REVOLUTION), 0, STEPS_PER_REVOLUTION);
        break;

    default:
        break;
    }

    // set motor setpoint
    motor.setSetpoint(motorSetpoint);

    // check for motor faults
    uint16_t fault = motor.getFault();
    if (fault != 0)
    {
        Serial.printf("Motor fault detected! Fault code: 0x%X\n", fault);
    }

    // send engine RPM and secondary RPM over CAN bus for telemetry
    CanMessage engineRpmMsg(CanDatabase::ENGINE_RPM.id, engineRPM);
    can.writeMessage(engineRpmMsg, 0);

    CanMessage motorSetpointMsg(CanDatabase::MOTOR_SETPOINT.id, motorSetpoint);
    can.writeMessage(motorSetpointMsg, 0);
}

int Controller::rpmToSetpoint(float rpm)
{

    if (rpm < ENGINE_ENGAGE_RPM) // if the rpm is less than the idle rpm
    {
        return MIN_MOTOR_SETPOINT;
    }
    // else if (rpm > MAX_RPM) // if the rpm is greater than the max rpm
    // {
    //     return MAX_SHEAVE_SETPOINT;
    // }
    else // P controller for RPM setpoint
    {

        float rpmError = ENGINE_IDEAL_RPM - rpm; // positive error means the rpm is too low

        float d_error = this->last_Error - rpmError; // Derivative error

        float d_setpoint = -rpmError * RPM_Kp + d_error * RPM_Kd; // negative because lower rpm means more negative sheve position position

        float low_setpoint = lerp(LOW_SHEAVE_SETPOINT, LOW_MAX_SETPOINT, (rpm - ENGINE_ENGAGE_RPM) / (ENGINE_MAX_RPM - ENGINE_ENGAGE_RPM));

        low_setpoint = clamp(low_setpoint, LOW_SHEAVE_SETPOINT, LOW_MAX_SETPOINT);

        this->last_Error = rpmError;

        // Serial.printf(">rpmError:%.2f\nd_error:%.2f\nlow_setpoint:%.2f\nd_setpoint:%.2f\n", rpmError, d_error, low_setpoint, d_setpoint);

        return clamp(this->motor.getPosition() + d_setpoint, low_setpoint, MAX_MOTOR_SETPOINT);
    }
}

// implement a 3 step homing routine where the motor first moves outwards until the limit switch is triggered, then moves inwards a little bit, then moves outwards again slowly until the limit switch is triggered again, and then sets the current position as the idle sheave position (0)
int Controller::homingRoutine()
{

    static bool firstLimitSwitchTriggered = false;
    static unsigned long limitSwitchTriggerTime = 0;

    bool limitSwitchState = digitalRead(LIMIT_SWITCH_PIN) == LOW;

    if (!firstLimitSwitchTriggered)
    {
        if (limitSwitchState)
        { // if the limit switch is already triggered, we are at the outer limit, so move inwards
            firstLimitSwitchTriggered = true;
            limitSwitchTriggerTime = millis();
            return this->motor.getPosition();
        }
        else
        { // if the limit switch is not triggered, move outwards until it is triggered
            return this->motor.getPosition() - 200;
        }
    }
    else if (firstLimitSwitchTriggered && (millis() - limitSwitchTriggerTime < 500))
    {// move outwards first trigger to ensure we are fully out of the limit switch, then move inwards a little bit
        return this->motor.getPosition() + 200; // move inwards a little bit
    }
    else if (firstLimitSwitchTriggered)
    { // after moving inwards, move outwards slowly until limit switch is triggered again
        if (limitSwitchState)
        { 
            // reset static variables for next homing routine
            firstLimitSwitchTriggered = false;
            limitSwitchTriggerTime = 0;
            this->motor.setHome(LIMIT_SWITCH_HOME_OFFSET);
            this->controlMode = POWER;              // switch to normal control mode after homing
            return this->motor.getPosition() + 800; // move outwards a little bit to ensure we are not triggering the switch anymore
        }
        else
        {                                          // if limit switch is not triggered, keep moving outwards slowly
            return this->motor.getPosition() - 50; // move outwards slowly
        }
    }

    return this->motor.getPosition(); // default return current position
}