// config.h
// This file contains configurations for the ECVT, such as pin assignments and constants.


// SECTION: CAN bus configurations

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4


// SECTION: Pulse counter configurations

#define PRIMARY_HALL_PIN GPIO_NUM_27
#define PRIMARY_COUNTER_ID PCNT_UNIT_1
#define PRIMARY_MAGNET_COUNT 3

#define SECONDARY_HALL_PIN GPIO_NUM_26
#define SECONDARY_COUNTER_ID PCNT_UNIT_3
#define SECONDARY_MAGNET_COUNT 6


// SECTION: Timer configurations

#define CONTROLLER_TIMER_RATE 50 // Control timer rate in milliseconds
#define MOTOR_TIMER_RATE 10       // Motor timer rate in milliseconds

// SECTION: Motor configurations


// SPI Pin Definitions
#define SPI_SDI_PIN         21    // GPIO21: SPI Slave Data In
#define SPI_SDO_PIN         22    // GPIO22: SPI Slave Data Out
#define SPI_SCK_PIN         17    // GPIO33: SPI Clock
#define SPI_nSCS_PIN        23    // GPIO23: SPI Chip Select

// Control Pin Definitions
#define nSLEEP_PIN          33    // GPIO19: nSLEEP control
#define ENABLE_PIN          25    // GPIO18: ENABLE control
#define DIR_PIN             13    // GPIO17: DIR control
#define STEP_PIN            16    // GPIO16: STEP control

#define RUN_MOTOR_CURRENT 200 // value from 0-255 to set the motor current, where 255 corresponds to the maximum current
#define HOLD_MOTOR_CURRENT 200 // value from 0-255 to set the motor current, where 255 corresponds to the maximum current
#define STEPS_PER_REVOLUTION 200 * 16 // 1.8 degree step angle = 200 steps per revolution, 16x microstepping = 3200 steps per revolution



// SECTION: Controller configurations

#define IDLE_SHEAVE_POSITION 0 // in units of encoder counts, where 0 is the idle position, and the positive direction is towards the engine

#define ENGINE_IDLE_RPM 1600
#define ENGINE_ENGAGE_RPM 2000
#define ENGINE_IDEAL_RPM 3000
#define ENGINE_MAX_RPM 3600

#define LOW_GEAR 3.6
#define HIGH_GEAR 0.9

// secondary rpm thresholds for gear shifting, calculated based on engine rpm and gear ratios
#define SLIP_SPEED ENGINE_ENGAGE_RPM / LOW_GEAR 
#define CRUISE_LOW ENGINE_IDEAL_RPM / LOW_GEAR
#define CRUISE_HIGH ENGINE_IDEAL_RPM / HIGH_GEAR

#define MIN_MOTOR_SETPOINT 0
#define MAX_MOTOR_SETPOINT STEPS_PER_REVOLUTION * 9 // 5 mm pitch leadscrew, 40mm travel = 10 revolutions, 25,600


#define LOW_SHEAVE_SETPOINT 20 // tune this, point where sheave starts to engage
#define LOW_MAX_SETPOINT 5000 // tune this, point sheave is fully engaged at low gear

#define clamp(x, min, max) (x < min ? min : x > max ? max : x)
#define lerp(a, b, k) (a + (b - a) * k)


#define RPM_Kp 3.0
#define RPM_Kd 0

