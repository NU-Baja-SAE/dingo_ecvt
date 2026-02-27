#include <Arduino.h>
#include <SPI.h>
#include "driver/rmt.h"  // for step pulse generation using RMT peripheral
#include "soc/rmt_reg.h" // for RMT register definitions

#include "DRV8462_REGMAP.h"


#define RMT_CHANNEL RMT_CHANNEL_0
#define MAX_PULSES 1000 // maximum number of pulses to send in one batch (adjust as needed)

class DRV8462
{
public:
    DRV8462();
    ~DRV8462();
    void begin();
    void enable();
    void disable();
    void moveSteps(int steps, int speed_hz);
    void stop();
    uint16_t readFault();
    void faultDetected();

private:
    rmt_item32_t pulse_buf[MAX_PULSES];
    SPIClass *spi;
    void spiWriteRegister(uint8_t address, uint16_t data);
    uint16_t spiReadRegister(uint8_t address);
    void setupRMT();
};