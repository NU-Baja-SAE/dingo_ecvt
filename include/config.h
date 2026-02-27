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

#define CONTROLLER_TIMER_RATE 50 // Control timer rate in milliseconds
#define MOTOR_TIMER_RATE 10       // Motor timer rate in milliseconds

// SECTION: Motor configurations


// SPI Pin Definitions
#define SPI_SDI_PIN         21    // GPIO23: SPI Slave Data In
#define SPI_SDO_PIN         22    // GPIO22: SPI Slave Data Out
#define SPI_SCK_PIN         17    // GPIO33: SPI Clock
#define SPI_nSCS_PIN        23    // GPIO21: SPI Chip Select

// Control Pin Definitions
#define nSLEEP_PIN          33    // GPIO19: nSLEEP control
#define ENABLE_PIN          25    // GPIO18: ENABLE control
#define DIR_PIN             13    // GPIO17: DIR control
#define STEP_PIN            16    // GPIO16: STEP control


#define STEPS_PER_REVOLUTION 200 // 1.8 degree step angle = 200 steps per revolution