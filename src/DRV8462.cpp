#include "DRV8462.h"
#include "config.h"


DRV8462::DRV8462()
{
    this->spi = new SPIClass(VSPI);
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

    // write to CTRL10 to set idle current to 10% (0.1 * 255 = 25.5 ~ 26)
    this->spiWriteRegister(SPI_CTRL10, 26); // set idle current to 10%
    // read CTRL10 to make sure idle current setting is correct
    uint16_t ctrl10Reg = this->spiReadRegister(SPI_CTRL10);
    if (ctrl10Reg != 26)
    {
        Serial.printf("Failed to set idle current! CTRL10 Register: 0x%X\n", ctrl10Reg);
        this->faultDetected();
    }

    // write to CTRL11 to set current to 10% (0.1 * 255 = 25.5 ~ 26)
    this->spiWriteRegister(SPI_CTRL11, 26); // set torque to 10%
    // read CTRL11 to make sure torque setting is correct
    uint16_t ctrl11Reg = this->spiReadRegister(SPI_CTRL11);
    if (ctrl11Reg != 26)
    {
        Serial.printf("Failed to set torque! CTRL11 Register: 0x%X\n", ctrl11Reg);
        this->faultDetected();
    }

    // Use internal Vref
    uint16_t ctrl13 = this->spiReadRegister(SPI_CTRL13);
    ctrl13 |= VREF_MASK; // set VREF bit
    this->spiWriteRegister(SPI_CTRL13, ctrl13);
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
 * @brief Writes a 16-bit value to a specified register address on the DRV8462 using SPI communication. It constructs the SPI command according to the protocol, initiates the SPI transaction, and checks for any faults indicated in the response.
 *
 * @param address The register address to write to (6 bits).
 * @param data The 8-bit data value to write to the register.
 */
void DRV8462::spiWriteRegister(uint8_t address, uint16_t data)
{
    // Implement SPI write operation to the DRV8462 registers
    volatile uint16_t reg_value = 0;

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
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK || dataMSB & OT_MASK)
    {
        this->faultDetected();
    }
}

/**
 * @brief Reads a 16-bit value from a specified register address on the DRV8462 using SPI communication. It constructs the SPI command according to the protocol, initiates the SPI transaction, checks for any faults indicated in the response, and returns the data read from the register.
 *
 * @param address The register address to read from (6 bits).
 * @return uint16_t The 16-bit data value read from the specified register.
 */
uint16_t DRV8462::spiReadRegister(uint8_t address)
{
    volatile uint16_t reg_value = 0;

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
    if (dataMSB & UVLO_MASK || dataMSB & CPUV_MASK || dataMSB & OCP_MASK || dataMSB & STL_MASK || dataMSB & OT_MASK)
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

void DRV8462::stop() {
    // stop the RMT step pulse generation
    rmt_tx_stop(RMT_CHANNEL);
}

uint16_t DRV8462::readFault()
{
    return this->spiReadRegister(SPI_FAULT);
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
 * @param steps 
 * @param speed_hz 
 */
void DRV8462::moveSteps(int steps, int speed_hz)
{
    int dir = steps >= 0 ? HIGH : LOW;
    digitalWrite(DIR_PIN, dir); // Set direction

    steps = abs(steps);
    
    if (steps == 0)
        return;
    if (steps > MAX_PULSES)
    {
        Serial.println("Warning: steps exceed MAX_PULSES, truncating to MAX_PULSES");
        steps = MAX_PULSES;
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

