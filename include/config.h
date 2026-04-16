// config.h
// This file contains configurations for the ECVT, such as pin assignments and constants.


// SECTION: CAN bus configurations

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4


// SECTION: Pulse counter configurations

#define PRIMARY_HALL_PIN GPIO_NUM_26
#define PRIMARY_COUNTER_ID PCNT_UNIT_1
#define PRIMARY_MAGNET_COUNT 3

#define SECONDARY_HALL_PIN GPIO_NUM_27
#define SECONDARY_COUNTER_ID PCNT_UNIT_3
#define SECONDARY_MAGNET_COUNT 6

// SECTION: Encoder configurations

#define ENCODER_A_PIN GPIO_NUM_34
#define ENCODER_B_PIN GPIO_NUM_35
#define ENCODER_COUNTER_ID PCNT_UNIT_2

// SECTION: Misc Pin configurations
#define LIN_POT_PIN GPIO_NUM_39
#define HOME_VOLTAGE 2270
#define MAX_VOLTAGE 4000


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

// value from 0-255 to set the motor current
#define RUN_MOTOR_CURRENT 120 
#define HOLD_MOTOR_CURRENT 80 
#define STEPS_PER_REVOLUTION 200 * 16 // 1.8 degree step angle = 200 steps per revolution, 16x microstepping = 3200 steps per revolution

// Auto torque configuration
#define ATQ_ENABLE 1
#define ATQ_TRQ_MIN_CURRENT 20 
#define ATQ_TRQ_MAX_CURRENT 80

// Use previously learned ATQ constants (production mode)
#define ATQ_USE_LEARNED_PARAMS 1
#define ATQ_LEARNED_CONST1 1280
#define ATQ_LEARNED_CONST2 91
#define ATQ_LEARNED_MIN_CURRENT_CODE 15
#define ATQ_LEARNED_STEP_CODE 2
#define ATQ_LEARNED_CYCLE_SELECT_CODE 3

// Auto torque learning routine configuration
#if ATQ_USE_LEARNED_PARAMS
#define ATQ_RUN_LEARNING_ON_STARTUP 0
#else
#define ATQ_RUN_LEARNING_ON_STARTUP 1
#endif
#define ATQ_LRN_MIN_CURRENT_CODE (RUN_MOTOR_CURRENT / 8) // Initial learning current = code * 8
#define ATQ_LRN_STEP_CODE 2                              // 00:128, 01:16, 10:32, 11:64
#define ATQ_LRN_CYCLE_SELECT_CODE 3                      // 00:8, 01:16, 10:24, 11:32 half-cycles
#define ATQ_ERROR_TRUNCATE_CODE 0                        // bits truncated from ATQ error before PD loop
#define ATQ_LRN_MOTION_SETTLE_MS 250                     // wait for steady-state motion before LRN_START
#define ATQ_LRN_TIMEOUT_MS 1500




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
#define MAX_MOTOR_SETPOINT STEPS_PER_REVOLUTION * 11 // 5 mm pitch leadscrew, 40mm travel = 10 revolutions, 25,600


#define LOW_SHEAVE_SETPOINT 20 // tune this, point where sheave starts to engage
#define LOW_MAX_SETPOINT 5000 // tune this, point sheave is fully engaged at low gear
#define LIMIT_SWITCH_HOME_OFFSET -2000

#define clamp(x, min, max) (x < min ? min : x > max ? max : x)
#define lerp(a, b, k) (a + (b - a) * k)


#define RPM_Kp 3.0
#define RPM_Kd 0



// rate limited debug printf
#define DEBUG_RATE_LIMIT_MS 100
#define debugPrintf(...) do { \
    static uint32_t lastPrintTime = 0; \
    uint32_t currentTime = millis(); \
    if (currentTime - lastPrintTime >= DEBUG_RATE_LIMIT_MS) { \
        Serial.printf(__VA_ARGS__); \
        lastPrintTime = currentTime; \
    } \
} while(0)
