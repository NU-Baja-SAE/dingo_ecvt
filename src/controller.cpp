#include "controller.h"
#include <Arduino.h>
#include "config.h"
#include "CanDatabase.h"

Controller::Controller() : motor(),
                           enginePulseCounter(PRIMARY_HALL_PIN, PRIMARY_COUNTER_ID, PRIMARY_MAGNET_COUNT),
                           can(CAN_TX_PIN, CAN_RX_PIN)
{
}

/**
 * @brief Start control timers, initialize I/O, and bring up CAN.
 */
void Controller::init()
{
    this->last_Error = 0.0f;
    this->resetHomingRoutine();
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
 * @brief Main control loop tick executed by the FreeRTOS timer.
 */
void Controller::timerCallback()
{
    // Determine motor setpoint based on mode
    int32_t motorSetpoint = 0;

    float engineRPM = enginePulseCounter.getRPM();
    this->brake_pressed = analogRead(BRAKE_PIN) > 1000;

    this->setMode();

    switch (this->controlMode)
    {
    case POWER:
    case TORQUE:
    case BRAKE_CHECK:
    case ACCELERATION:
        motorSetpoint = this->rpmToSetpoint(engineRPM);
        if (this->brake_pressed) {
            motorSetpoint = HOME_POSITION;
        }
        break;

    case DEBUG:
        motorSetpoint = constrain(map(millis(), 0, 1000, 0, STEPS_PER_REVOLUTION), 0, STEPS_PER_REVOLUTION);
        break;

    case HOMING:
        motorSetpoint = this->homingRoutine();
        break;

    case BRAKE:
        motorSetpoint = HOME_POSITION;
    default:
        break;
    }

    // Apply setpoint to the motor controller.
    motor.setSetpoint(motorSetpoint);

    // Check for motor faults reported by the driver.
    uint16_t fault = motor.getFault();
    if (fault != 0)
    {
        Serial.printf("Motor fault detected! Fault code: 0x%X\n", fault);
    }

    this->sendCan();
}

int Controller::rpmToSetpoint(float rpm)
{
    float targetRPM = 0.0f;
    switch (this->controlMode)
    {
    case POWER:
        targetRPM = ENGINE_IDEAL_RPM_POWER;
        break;
    case TORQUE:
        targetRPM = ENGINE_IDEAL_RPM_TORQUE;
        break;
    default:
        targetRPM = ENGINE_IDEAL_RPM_POWER;
        break;
    }

    if (rpm < ENGINE_ENGAGE_RPM)
    {
        return IDLE_MOTOR_SETPOINT;
    }
    else
    {

        float rpmError = targetRPM - rpm; // positive error means the rpm is too low

        float d_error = this->last_Error - rpmError; // Derivative error

        float d_setpoint = -rpmError * RPM_Kp + d_error * RPM_Kd; // negative because lower rpm means more negative sheve position position

        float low_setpoint = lerp(IDLE_MOTOR_SETPOINT, LOW_MAX_SETPOINT, (rpm - ENGINE_ENGAGE_RPM) / (ENGINE_MAX_RPM - ENGINE_ENGAGE_RPM));

        low_setpoint = clamp(low_setpoint, IDLE_MOTOR_SETPOINT, LOW_MAX_SETPOINT);

        this->last_Error = rpmError;
        if (this->controlMode == BRAKE_CHECK) {
            return clamp(this->motor.getPosition() + d_setpoint, low_setpoint, MAX_MOTOR_SETPOINT_BRAKE_MODE);
        }
        return clamp(this->motor.getPosition() + d_setpoint, low_setpoint, MAX_MOTOR_SETPOINT);
    }
}




#define HOME_SPEED 400
#define HOME_SPEED_SLOW 50

/**
 * @brief 3-step homing routine to find the mechanical limit switch.
 * @return Motor setpoint for the current homing phase.
 */
int Controller::homingRoutine()
{
    // Limit switch is active high.
    bool limitSwitchState = analogRead(LIMIT_SWITCH_PIN) > 2000;

    if (!this->homingFirstTrigger)
    {
        if (limitSwitchState)
        {
            this->homingFirstTrigger = true;
            this->homingTriggerTime = millis();
            return this->motor.getPosition();
        }
        else
        {
            return this->motor.getPosition() - HOME_SPEED;
        }
    }
    else if (this->homingFirstTrigger && (millis() - this->homingTriggerTime < 400))
    {
        // Move inward briefly to clear the switch, then return outward slowly.
        return this->motor.getPosition() + HOME_SPEED;
    }
    else if (this->homingFirstTrigger)
    {
        if (limitSwitchState)
        {
            this->motor.setHome(LIMIT_SWITCH_POS);
            this->resetHomingRoutine();
            this->controlMode = POWER;
            // Move outward to clear the switch after homing completes.
            return this->motor.getPosition() + 800;
        }
        else
        {
            return this->motor.getPosition() - HOME_SPEED_SLOW;
        }
    }

    return this->motor.getPosition();
}

void Controller::resetHomingRoutine()
{
    this->homingFirstTrigger = false;
    this->homingTriggerTime = 0;
}



/**
 * @brief Send telemetry frames to the CAN bus.
 */
void Controller::sendCan() {
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

    CanMessage brakeStateMsg(CanDatabase::SECONDARY_RPM.id, (float)(this->brake_pressed ? 1 : 0));
    ret = can.writeMessage(brakeStateMsg, 0);
    if (ret != ESP_OK)    {
        #ifdef CAN_DEBUG
        Serial.printf("Failed to send BRAKE_STATE message. Error code: %s\n", esp_err_to_name(ret));
        #endif
    }

    #ifdef CAN_DEBUG
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("CAN bus status - msgs_to_tx: %d, msgs_to_rx: %d, bus_state: %d\n", status.msgs_to_tx, status.msgs_to_rx, status.state);
    }
    #endif
}

/**
 * @brief Read inbound CAN messages for brake and mode data.
 */
void Controller::readCan() {
    CanMessage message;
    while (can.readMessage(message, 0) == ESP_OK) {
        if (message.getId() == CanDatabase::BRAKE_POT.id && message.getDataType() == CanDatabase::BRAKE_POT.type) {
            this->brake_pos = message.getFloat();
            #ifdef CAN_DEBUG
            Serial.printf(">BRAKE_POT:%.2f\n", this->brake_pos);
            #endif

            if (this->brake_pos > BRAKE_THRESHOLD) {
                this->controlMode = BRAKE;
            } else {
                this->controlMode = POWER;  
            }
        } else if (message.getId() == CanDatabase::LINEAR_SPEED.id && message.getDataType() == CanDatabase::LINEAR_SPEED.type) {
            float linearSpeed = message.getFloat();
            #ifdef CAN_DEBUG
            Serial.printf(">LINEAR_SPEED:%.2f\n", linearSpeed);
            #endif

        }

    }
}


/**
 * @brief Set control mode from the manual selector input.
 */
void Controller::setMode() {
    static ControlMode lastMode = HOMING;

    if (this->controlMode == HOMING) {
        return;
    }

    if (analogRead(MANUAL_MODE_PIN) < POWER_MODE_THRESHOLD) {
        this->controlMode = POWER;
    } else if (analogRead(MANUAL_MODE_PIN) < TORQUE_MODE_THRESHOLD) {
        this->controlMode = TORQUE;
    } else if (analogRead(MANUAL_MODE_PIN) < ACCELERATION_MODE_THRESHOLD) {
        this->controlMode = ACCELERATION;
    } else if (analogRead(MANUAL_MODE_PIN) < BRAKE_CHECK_MODE_THRESHOLD) {
        this->controlMode = BRAKE_CHECK;
    } else if (lastMode != HOMING) {
        this->resetHomingRoutine();
        this->controlMode = HOMING;
    }
    lastMode = this->controlMode;
}
