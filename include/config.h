// config.h
// This file contains configurations for the ECVT, such as pin assignments and constants.


// SECTION: CAN bus configurations

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4


// SECTION: Pulse counter configurations

#define PRIMARY_HALL_PIN GPIO_NUM_26
#define PRIMARY_COUNTER_ID PCNT_UNIT_1
#define PRIMARY_MAGNET_COUNT 6

#define SECONDARY_HALL_PIN GPIO_NUM_27
#define SECONDARY_COUNTER_ID PCNT_UNIT_3
#define SECONDARY_MAGNET_COUNT 6


// SECTION: Timer configurations

#define CONTROLLER_TIMER_RATE 1000 // Control timer rate in milliseconds
#define MOTOR_TIMER_RATE 500       // Motor timer rate in milliseconds