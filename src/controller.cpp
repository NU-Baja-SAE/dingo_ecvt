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

    pinMode(LIMIT_SWITCH_PIN, INPUT);
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
    bool limitSwitchState = analogRead(LIMIT_SWITCH_PIN) > 2000;

    switch (this->controlMode)
    {
    case POWER:
        if (limitSwitchState)
        {
            this->motor.setHome(LIMIT_SWITCH_POS);
        }
        motorSetpoint = this->rpmToSetpoint(engineRPM);
        break;

    case DEBUG:
        motorSetpoint = constrain(map(millis(), 0, 1000, 0, STEPS_PER_REVOLUTION), 0, STEPS_PER_REVOLUTION);
        break;

    case HOMING:
        motorSetpoint = this->homingRoutine();
        break;

    case BRAKE:
        motorSetpoint = IDLE_SHEAVE_POSITION;
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

    this->sendCan();
    this->readCan();
}

int Controller::rpmToSetpoint(float rpm)
{

    if (rpm < ENGINE_ENGAGE_RPM) // if the rpm is less than the idle rpm
    {
        return MIN_MOTOR_SETPOINT;
    }
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




#define HOME_SPEED 400 
#define HOME_SPEED_SLOW 50
// implement a 3 step homing routine where the motor first moves outwards until the limit switch is triggered, then moves inwards a little bit, then moves outwards again slowly until the limit switch is triggered again, and then sets the current position as the idle sheave position (0)
int Controller::homingRoutine()
{

    static bool firstLimitSwitchTriggered = false;
    static unsigned long limitSwitchTriggerTime = 0;

    // check limit switch state, active high
    bool limitSwitchState = analogRead(LIMIT_SWITCH_PIN) > 2000; // limit switch is triggered when the pin reads LOW due to pull-down configuration (normally closed switch to power)

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
            return this->motor.getPosition() - HOME_SPEED;
        }
    }
    else if (firstLimitSwitchTriggered && (millis() - limitSwitchTriggerTime < 400))
    {                                           // move outwards for 0.5 seconds after first trigger to ensure we are fully out of the limit switch, then move inwards a little bit
        return this->motor.getPosition() + HOME_SPEED; // move inwards a little bit
    }
    else if (firstLimitSwitchTriggered)
    { // after moving inwards, move outwards slowly until limit switch is triggered again
        if (limitSwitchState)
        { // if limit switch is triggered again, we are at the home position
            // set current position as idle sheave position (0)
            this->motor.setHome(LIMIT_SWITCH_POS);
            // reset static variables for next homing routine
            firstLimitSwitchTriggered = false;
            limitSwitchTriggerTime = 0;
            this->controlMode = POWER;              // switch to normal control mode after homing
            return this->motor.getPosition() + 800; // move outwards a little bit to ensure we are not triggering the switch anymore
        }
        else
        {                                          // if limit switch is not triggered, keep moving outwards slowly
            return this->motor.getPosition() - HOME_SPEED_SLOW; // move outwards slowly
        }
    }

    return this->motor.getPosition(); // default return current position
}



void Controller::sendCan() {
    // send engine RPM and secondary RPM over CAN bus for telemetry
    CanMessage engineRpmMsg(CanDatabase::ENGINE_RPM.id, this->enginePulseCounter.getFilteredRPM());
    esp_err_t ret = can.writeMessage(engineRpmMsg, 0);
    
    if (ret != ESP_OK)
    {
        #ifdef CAN_DEBUG
        Serial.printf("Failed to send ENGINE_RPM message. Error code: %s\n", esp_err_to_name(ret));
        #endif
    }

    CanMessage motorSetpointMsg(CanDatabase::MOTOR_SETPOINT.id, this->motor.getSetpoint());
    ret = can.writeMessage(motorSetpointMsg, 0);
    if (ret != ESP_OK)
    {
        #ifdef CAN_DEBUG
        Serial.printf("Failed to send MOTOR_SETPOINT message. Error code: %s\n", esp_err_to_name(ret));
        #endif
    }

    CanMessage motorPositionMsg(CanDatabase::MOTOR_POSITION.id, this->motor.getPosition());
    ret = can.writeMessage(motorPositionMsg, 0);
    if (ret != ESP_OK)    {
        #ifdef CAN_DEBUG
        Serial.printf("Failed to send MOTOR_POSITION message. Error code: %s\n", esp_err_to_name(ret));
        #endif
    }

    #ifdef CAN_DEBUG
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("CAN bus status - msgs_to_tx: %d, msgs_to_rx: %d, bus_state: %d\n", status.msgs_to_tx, status.msgs_to_rx, status.state);
    }
    #endif
}

void Controller::readCan() {
    // read CAN messages and update control mode accordingly
    CanMessage message;
    while (can.readMessage(message, 0) == ESP_OK) {
        if (message.getId() == CanDatabase::BRAKE_POT.id && message.getDataType() == CanDatabase::BRAKE_POT.type) {
            this->brake_pos = message.getFloat();
            #ifdef CAN_DEBUG
            Serial.printf(">BRAKE_POT:%.2f\n", this->brake_pos);
            #endif

            // if brake is pressed more than 50%, go to brake mode, otherwise go to power mode
            if (this->brake_pos > BRAKE_THRESHOLD) {
                this->controlMode = BRAKE;
            } else {
                this->controlMode = POWER;  
            }
        }
    }
}
