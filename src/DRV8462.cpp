#include "DRV8462.h"
#include "config.h"

DRV8462::DRV8462()
{
    this->spi = new SPIClass(VSPI);
    this->atqLearningPending = false;
    this->atqLearningInProgress = false;
    this->atqLearningComplete = false;
    this->atqLearningMotionStartMs = 0;
    this->atqLearningStartMs = 0;
}

DRV8462::~DRV8462()
{
    delete this->spi;
}

void DRV8462::setupAutoTorque()
{
    if (!ATQ_ENABLE)
    {
        return;
    }

    this->spiWriteRegister(SPI_ATQ_CTRL11, ATQ_TRQ_MIN_CURRENT);
    this->spiWriteRegister(SPI_ATQ_CTRL12, ATQ_TRQ_MAX_CURRENT);

    if (this->spiReadRegister(SPI_ATQ_CTRL11) != ATQ_TRQ_MIN_CURRENT)
    {
        Serial.println("Failed to set ATQ_TRQ_MIN in ATQ_CTRL11");
        this->faultDetected();
    }

    if (this->spiReadRegister(SPI_ATQ_CTRL12) != ATQ_TRQ_MAX_CURRENT)
    {
        Serial.println("Failed to set ATQ_TRQ_MAX in ATQ_CTRL12");
        this->faultDetected();
    }

    uint16_t atq_ctrl15 = 0;

#if ATQ_USE_LEARNED_PARAMS
    uint16_t atq_ctrl2 = (ATQ_LEARNED_CONST1 & 0xFF);
    uint16_t atq_ctrl3 = ((ATQ_LEARNED_CONST1 >> 8) & 0x07);
    uint16_t atq_ctrl4 = (((ATQ_LEARNED_MIN_CURRENT_CODE & 0x1F) << 3) & ATQ_LRN_MIN_CURRENT_MASK) |
                         ((ATQ_LEARNED_CONST2 >> 8) & ATQ_LRN_CONST2_MSB_MASK);
    uint16_t atq_ctrl5 = (ATQ_LEARNED_CONST2 & 0xFF);

    atq_ctrl15 = (((ATQ_ERROR_TRUNCATE_CODE & 0x0F) << 4) & ATQ_ERROR_TRUNCATE_MASK) |
                 (((ATQ_LEARNED_STEP_CODE & 0x03) << 2) & ATQ_LRN_STEP_FIELD_MASK) |
                 ((ATQ_LEARNED_CYCLE_SELECT_CODE & 0x03) & ATQ_LRN_CYCLE_FIELD_MASK);

    this->spiWriteRegister(SPI_ATQ_CTRL2, atq_ctrl2);
    this->spiWriteRegister(SPI_ATQ_CTRL3, atq_ctrl3);
    this->spiWriteRegister(SPI_ATQ_CTRL4, atq_ctrl4);
    this->spiWriteRegister(SPI_ATQ_CTRL5, atq_ctrl5);
    this->spiWriteRegister(SPI_ATQ_CTRL15, atq_ctrl15);

    Serial.printf("ATQ learned params loaded. CONST1=%u CONST2=%u\n", ATQ_LEARNED_CONST1, ATQ_LEARNED_CONST2);
#else
    uint16_t atq_ctrl4 = this->spiReadRegister(SPI_ATQ_CTRL4);
    atq_ctrl4 = (atq_ctrl4 & ATQ_LRN_CONST2_MSB_MASK) |
                (((ATQ_LRN_MIN_CURRENT_CODE & 0x1F) << 3) & ATQ_LRN_MIN_CURRENT_MASK);
    this->spiWriteRegister(SPI_ATQ_CTRL4, atq_ctrl4);

    atq_ctrl15 = (((ATQ_ERROR_TRUNCATE_CODE & 0x0F) << 4) & ATQ_ERROR_TRUNCATE_MASK) |
                 (((ATQ_LRN_STEP_CODE & 0x03) << 2) & ATQ_LRN_STEP_FIELD_MASK) |
                 ((ATQ_LRN_CYCLE_SELECT_CODE & 0x03) & ATQ_LRN_CYCLE_FIELD_MASK);
    this->spiWriteRegister(SPI_ATQ_CTRL15, atq_ctrl15);
#endif

    if (this->spiReadRegister(SPI_ATQ_CTRL15) != atq_ctrl15)
    {
        Serial.println("Failed to set ATQ learning parameters in ATQ_CTRL15");
        this->faultDetected();
    }

    uint16_t atq_ctrl10 = this->spiReadRegister(SPI_ATQ_CTRL10);
    atq_ctrl10 |= ATQ_EN_MASK;
    this->spiWriteRegister(SPI_ATQ_CTRL10, atq_ctrl10);

    uint16_t atq_ctrl10_verify = this->spiReadRegister(SPI_ATQ_CTRL10);
    if ((atq_ctrl10_verify & ATQ_EN_MASK) == 0)
    {
        Serial.println("Failed to enable auto torque (ATQ_EN)");
        this->faultDetected();
    }

#if ATQ_RUN_LEARNING_ON_STARTUP
    this->atqLearningPending = true;
    this->atqLearningInProgress = false;
    this->atqLearningComplete = false;
    this->atqLearningMotionStartMs = 0;
    Serial.println("ATQ learning armed: will start on first motor motion");
#elif ATQ_USE_LEARNED_PARAMS
    Serial.println("ATQ using saved learned parameters (learning routine disabled)");
#endif
}

void DRV8462::serviceAutoTorqueLearning(bool motorIsStepping)
{
#if ATQ_RUN_LEARNING_ON_STARTUP
    if (!ATQ_ENABLE || this->atqLearningComplete)
    {
        return;
    }

    if (this->atqLearningPending && motorIsStepping)
    {
        if (this->atqLearningMotionStartMs == 0)
        {
            this->atqLearningMotionStartMs = millis();
            return;
        }

        if ((millis() - this->atqLearningMotionStartMs) < ATQ_LRN_MOTION_SETTLE_MS)
        {
            return;
        }

        uint16_t atq_ctrl10 = this->spiReadRegister(SPI_ATQ_CTRL10);
        atq_ctrl10 |= LRN_START_MASK;
        this->spiWriteRegister(SPI_ATQ_CTRL10, atq_ctrl10);

        uint16_t atq_ctrl10_verify = this->spiReadRegister(SPI_ATQ_CTRL10);
        if ((atq_ctrl10_verify & LRN_START_MASK) == 0)
        {
            Serial.println("Failed to set ATQ learning start (LRN_START)");
            this->faultDetected();
            this->atqLearningMotionStartMs = 0;
            return;
        }

        this->atqLearningPending = false;
        this->atqLearningInProgress = true;
        this->atqLearningStartMs = millis();
        Serial.println("ATQ learning started");
    }
    else if (this->atqLearningPending && !motorIsStepping)
    {
        this->atqLearningMotionStartMs = 0;
    }

    if (!this->atqLearningInProgress)
    {
        return;
    }

    uint16_t diag2 = this->spiReadRegister(SPI_DIAG2);
    uint16_t atq_ctrl10 = this->spiReadRegister(SPI_ATQ_CTRL10);
    if (((diag2 & ATQ_LRN_DONE_MASK) != 0) && ((atq_ctrl10 & LRN_START_MASK) == 0))
    {
        this->atqLearningInProgress = false;
        this->atqLearningComplete = true;

        uint16_t atq_ctrl2 = this->spiReadRegister(SPI_ATQ_CTRL2);
        uint16_t atq_ctrl3 = this->spiReadRegister(SPI_ATQ_CTRL3);
        uint16_t atq_ctrl4_learn = this->spiReadRegister(SPI_ATQ_CTRL4);
        uint16_t atq_ctrl5 = this->spiReadRegister(SPI_ATQ_CTRL5);

        uint16_t atq_lrn_const1 = ((atq_ctrl3 & 0x07) << 8) | (atq_ctrl2 & 0xFF);
        uint16_t atq_lrn_const2 = ((atq_ctrl4_learn & ATQ_LRN_CONST2_MSB_MASK) << 8) | (atq_ctrl5 & 0xFF);

        Serial.printf("ATQ learning done. CONST1=%u CONST2=%u\n", atq_lrn_const1, atq_lrn_const2);
        Serial.printf("ATQ regs: CTRL2=0x%02X CTRL3=0x%02X CTRL4=0x%02X CTRL5=0x%02X\n",
                      atq_ctrl2,
                      atq_ctrl3,
                      atq_ctrl4_learn,
                      atq_ctrl5);
        return;
    }

    if ((millis() - this->atqLearningStartMs) > ATQ_LRN_TIMEOUT_MS)
    {
        if ((atq_ctrl10 & LRN_START_MASK) != 0)
        {
            atq_ctrl10 &= ~LRN_START_MASK;
            this->spiWriteRegister(SPI_ATQ_CTRL10, atq_ctrl10);
        }

        this->atqLearningInProgress = false;
        this->atqLearningPending = true;
        this->atqLearningMotionStartMs = 0;
        Serial.println("Auto-torque learning timeout; will retry on next motor motion");
    }
#else
    (void)motorIsStepping;
#endif
}

/**
 * @brief Sets up the DRV8462 driver by initializing SPI communication, configuring control registers, and setting up the RMT peripheral for step pulse generation.
 *
 */
void DRV8462::begin()
{

    this->spi->begin(SPI_SCK_PIN, SPI_SDO_PIN, SPI_SDI_PIN, SPI_nSCS_PIN);

    pinMode(nSLEEP_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(SPI_nSCS_PIN, OUTPUT);

    digitalWrite(nSLEEP_PIN, HIGH);   // Wake up the driver
    digitalWrite(SPI_nSCS_PIN, HIGH); // Set SS high
    // wait t_wake = 1.5 ms
    delayMicroseconds(2000);
    digitalWrite(ENABLE_PIN, LOW); // disable the driver

    this->setupRMT();

    // Read fault register
    uint16_t faultReg = this->spiReadRegister(SPI_FAULT);
    if (faultReg != 0)
    {
        Serial.printf("Fault detected on startup! Fault Register: 0x%X\n", faultReg);
        this->faultDetected();
    }

    // enable open load detection
    uint16_t ctrl9 = this->spiReadRegister(SPI_CTRL9);
    ctrl9 |= OLD_MASK; // set OLD bit
    this->spiWriteRegister(SPI_CTRL9, ctrl9);

    // write to CTRL10 to set idle current 
    this->spiWriteRegister(SPI_CTRL10, HOLD_MOTOR_CURRENT); 
    // read CTRL10 to make sure idle current setting is correct
    uint16_t ctrl10Reg = this->spiReadRegister(SPI_CTRL10);
    if (ctrl10Reg != HOLD_MOTOR_CURRENT)
    {
        Serial.printf("Failed to set idle current! CTRL10 Register: 0x%X\n", ctrl10Reg);
        this->faultDetected();
    }

    // write to CTRL11 to set current limit
    this->spiWriteRegister(SPI_CTRL11, RUN_MOTOR_CURRENT); 
    // read CTRL11 to make sure torque setting is correct
    uint16_t ctrl11Reg = this->spiReadRegister(SPI_CTRL11);
    if (ctrl11Reg != RUN_MOTOR_CURRENT)
    {
        Serial.printf("Failed to set torque! CTRL11 Register: 0x%X\n", ctrl11Reg);
        this->faultDetected();
    }

    // Use internal Vref
    uint16_t ctrl13 = this->spiReadRegister(SPI_CTRL13);
    ctrl13 |= VREF_MASK; // set VREF bit
    this->spiWriteRegister(SPI_CTRL13, ctrl13);

    this->setupAutoTorque();
}

/**
 * @brief Enables the DRV8462 driver by setting the appropriate control register bit and driving the enable pin high.
 *
 */
void DRV8462::enable()
{
    uint16_t ctrl1Reg = this->spiReadRegister(SPI_CTRL1);
    ctrl1Reg |= EN_OUT_MASK; // set EN_OUT bit
    this->spiWriteRegister(SPI_CTRL1, ctrl1Reg);

    digitalWrite(ENABLE_PIN, HIGH); // enable the driver
}

/**
 * @brief Writes a 8-bit value to a specified register address on the DRV8462 using SPI communication. It constructs the SPI command according to the protocol, initiates the SPI transaction, and checks for any faults indicated in the response.
 *
 * @param address The register address to write to (6 bits).
 * @param data The 8-bit data value to write to the register.
 */
void DRV8462::spiWriteRegister(uint8_t address, uint16_t data)
{
    // Implement SPI write operation to the DRV8462 registers
    uint16_t reg_value = 0;

    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); // Adding register address value
    reg_value |= ((data << SPI_DATA_POS) & SPI_DATA_MASK);          // Adding data value

    spi->beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE1));
    digitalWrite(spi->pinSS(), LOW); // pull SS low to prep other end for transfer

    uint16_t received = spi->transfer16(reg_value);

    digitalWrite(spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    spi->endTransaction();

    uint8_t dataMSB = (received >> 8) & 0xFF;

    // check that first 2 bits are set
    if ((dataMSB & 0xC0) != 0xC0)
    {
        this->faultDetected();
    }

    // check fault bits of MSB
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK)
    {
        this->faultDetected();
    }
}

/**
 * @brief Reads a 8-bit value from a specified register address on the DRV8462 using SPI communication. It constructs the SPI command according to the protocol, initiates the SPI transaction, checks for any faults indicated in the response, and returns the data read from the register.
 *
 * @param address The register address to read from (6 bits).
 * @return uint16_t The 16-bit data value read from the specified register.
 */
uint16_t DRV8462::spiReadRegister(uint8_t address)
{
    uint16_t reg_value = 0;

    reg_value |= ((address << SPI_ADDRESS_POS) & SPI_ADDRESS_MASK); // Configure register address value
    reg_value |= SPI_RW_BIT_MASK;                                   // Set R/W bit

    spi->beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE1));
    digitalWrite(spi->pinSS(), LOW); // pull SS low to prep other end for transfer

    uint16_t received = spi->transfer16(reg_value);
    digitalWrite(spi->pinSS(), HIGH); // pull ss high to signify end of data transfer
    spi->endTransaction();

    uint8_t dataMSB = (received >> 8) & 0xFF;
    uint8_t dataLSB = received & 0xFF;

    // check that first 2 bits are set
    if ((dataMSB & 0xC0) != 0xC0)
    {
        this->faultDetected();
    }
    // check fault bits of MSB
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK)
    {
        this->faultDetected();
    }

    reg_value = ((((dataMSB << 8) | dataLSB) & SPI_DATA_MASK) >> SPI_DATA_POS); // complete data
    return (reg_value);
}

void DRV8462::faultDetected()
{
    Serial.println("Fault detected!");
}

void DRV8462::disable()
{
    uint16_t ctrl1Reg = this->spiReadRegister(SPI_CTRL1);
    ctrl1Reg &= ~EN_OUT_MASK; // clear EN_OUT bit
    this->spiWriteRegister(SPI_CTRL1, ctrl1Reg);

    digitalWrite(ENABLE_PIN, LOW); // disable the driver
}

void DRV8462::stop()
{
    // stop the RMT step pulse generation
    rmt_tx_stop(RMT_CHANNEL);
}

uint16_t DRV8462::readFault()
{
    return this->spiReadRegister(SPI_FAULT);
}

uint16_t DRV8462::readDiag2()
{
    return this->spiReadRegister(SPI_DIAG2);
}

bool DRV8462::isAtqLearningDone()
{
    return (this->readDiag2() & ATQ_LRN_DONE_MASK) != 0;
}

void DRV8462::printAtqLearnedParameters()
{
    uint16_t atq_ctrl2 = this->spiReadRegister(SPI_ATQ_CTRL2);
    uint16_t atq_ctrl3 = this->spiReadRegister(SPI_ATQ_CTRL3);
    uint16_t atq_ctrl4 = this->spiReadRegister(SPI_ATQ_CTRL4);
    uint16_t atq_ctrl5 = this->spiReadRegister(SPI_ATQ_CTRL5);
    uint16_t atq_ctrl15 = this->spiReadRegister(SPI_ATQ_CTRL15);

    uint16_t atq_lrn_const1 = ((atq_ctrl3 & 0x07) << 8) | (atq_ctrl2 & 0xFF);
    uint16_t atq_lrn_const2 = ((atq_ctrl4 & ATQ_LRN_CONST2_MSB_MASK) << 8) | (atq_ctrl5 & 0xFF);

    uint16_t atq_lrn_min_current = (atq_ctrl4 & ATQ_LRN_MIN_CURRENT_MASK) >> 3;
    uint16_t atq_lrn_step = (atq_ctrl15 & ATQ_LRN_STEP_FIELD_MASK) >> 2;
    uint16_t atq_lrn_cycle_select = (atq_ctrl15 & ATQ_LRN_CYCLE_FIELD_MASK);

    Serial.println("=== ATQ LEARNED PARAMETERS ===");
    Serial.printf("ATQ_LRN_CONST1=%u\n", atq_lrn_const1);
    Serial.printf("ATQ_LRN_CONST2=%u\n", atq_lrn_const2);
    Serial.printf("ATQ_LRN_MIN_CURRENT(code)=%u\n", atq_lrn_min_current);
    Serial.printf("ATQ_LRN_STEP(code)=%u\n", atq_lrn_step);
    Serial.printf("ATQ_LRN_CYCLE_SELECT(code)=%u\n", atq_lrn_cycle_select);
    Serial.printf("ATQ_CTRL2=0x%02X ATQ_CTRL3=0x%02X ATQ_CTRL4=0x%02X ATQ_CTRL5=0x%02X ATQ_CTRL15=0x%02X\n",
                  atq_ctrl2,
                  atq_ctrl3,
                  atq_ctrl4,
                  atq_ctrl5,
                  atq_ctrl15);
    Serial.println("==============================");
}

void DRV8462::setupRMT()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)STEP_PIN;
    config.mem_block_num = 1;
    config.clk_div = 80; // 80MHz / 80 = 1MHz resolution (1 tick = 1 microsecond)
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(RMT_CHANNEL, 0, 0);
}

/**
 * @brief Moves the motor a specified number of steps at a given speed in Hz. It calculates the pulse duration based on the desired speed, constructs the RMT items for the step pulses, and sends them using the RMT peripheral. The direction pin is set according to the sign of the steps parameter.
 *
 * @param steps number of steps to move (positive for one direction, negative for the other)
 * @param speed_hz speed of the steps in Hz (steps per second)
 */
void DRV8462::moveSteps(int steps, int speed_hz)
{
    this->serviceAutoTorqueLearning((steps != 0) && (speed_hz > 0));

    // check if last command is still executing, if so, stop it before sending new command
    rmt_channel_status_result_t status;
    rmt_get_channel_status(&status);
    if (status.status[RMT_CHANNEL] == RMT_CHANNEL_BUSY)
    {
        // Serial.println("Previous command still executing, stopping it before sending new command.");
        rmt_tx_stop(RMT_CHANNEL);
    }

    int dir = steps >= 0 ? LOW : HIGH;
    digitalWrite(DIR_PIN, dir); // Set direction

    steps = abs(steps);

    if (steps == 0)
        return;
    if (steps > MAX_PULSES)
    {
        Serial.println("Warning: steps exceed MAX_PULSES, truncating to MAX_PULSES");
        steps = MAX_PULSES;
    }

    // Validate speed to avoid division by zero and unreasonable values
    if (speed_hz <= 0)
    {
        Serial.println("Error: speed_hz must be greater than 0");
        return;
    }

    // Calculate pulse duration in microseconds
    // For a 50% duty cycle: Period = 1,000,000 / speed_hz
    uint32_t duration_us = 1000000 / speed_hz / 2;

    // Limit duration to RMT max (15-bit value, max 32767)
    if (duration_us > 32767)
        duration_us = 32767;

    // Create the pulse item: High for duration_us, Low for duration_us
    rmt_item32_t pulse = {{{(uint16_t)duration_us, 1, (uint16_t)duration_us, 0}}};

    for (int i = 0; i < steps; i++)
    {
        this->pulse_buf[i] = pulse;
    }

    // Send the items (this is non-blocking)
    rmt_write_items(RMT_CHANNEL, this->pulse_buf, steps, false);
}



void DRV8462::pullStepLow() {
    if (this->isHigh) {
        gpio_set_level(gpio_num_t(STEP_PIN), 0); // Fast ESP-IDF call
        this->isHigh = false;
    }
}

void DRV8462::step(bool dir) {
    // Set direction
    gpio_set_level(gpio_num_t(DIR_PIN), dir ? 1 : 0); 
    
    // The DRV8462 requires a minimum direction setup time (typically 200ns).
    // At 240MHz, a few NOPs or the function call overhead itself is usually enough,
    // but keep an eye on this if the motor steps the wrong way occasionally.

    // Fire the step pulse
    gpio_set_level(gpio_num_t(STEP_PIN), 1); 
    this->isHigh = true;

    
}