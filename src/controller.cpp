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
    this->readLinPot();

    switch (this->controlMode)
    {
    case POWER:
        /* code */
        motorSetpoint = this->rpmToSetpoint(engineRPM);
        break;
    case DEBUG:

        motorSetpoint = constrain(map(millis(), 0, 1000, 0, STEPS_PER_REVOLUTION), 0, STEPS_PER_REVOLUTION);
        break;

    default:
        break;
    }

    // set motor setpoint

    motor.setPosition(this->lin_pot_pos); // for testing, set motor position to lin pot position
    motor.setSetpoint(motorSetpoint);

    // check for motor faults
    uint16_t fault = motor.getFault();
    if (fault != 0)
    {
        Serial.printf("Motor fault detected! Fault code: 0x%X\n", fault);
    }

    // send engine RPM and secondary RPM over CAN bus for telemetry
    CanMessage engineRpmMsg(CanDatabase::ENGINE_RPM.id, engineRPM);
    esp_err_t ret = can.writeMessage(engineRpmMsg, 0);
    if (ret != ESP_OK)
    {
        Serial.printf("Failed to send ENGINE_RPM message. Error code: %s\n", esp_err_to_name(ret));
    }
    CanMessage motorSetpointMsg(CanDatabase::MOTOR_SETPOINT.id, motorSetpoint);
    ret = can.writeMessage(motorSetpointMsg, 0);
    if (ret != ESP_OK)
    {
        Serial.printf("Failed to send MOTOR_SETPOINT message. Error code: %s\n", esp_err_to_name(ret));
    }

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("CAN bus status - msgs_to_tx: %d, msgs_to_rx: %d, bus_state: %d\n", status.msgs_to_tx, status.msgs_to_rx, status.state);
    }
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

// reads lin pot and converts the voltage to a position in steps
void Controller::readLinPot()
{
    // read linear potentiometer value and convert to position in steps
    int linPotValue = analogRead(LIN_POT_PIN);
    int new_lin_pot_pos = map(linPotValue, HOME_VOLTAGE, MAX_VOLTAGE, LIMIT_SWITCH_HOME_OFFSET, MAX_MOTOR_SETPOINT);
    this->lin_pot_pos = linPotFilter.filter(new_lin_pot_pos);
}
