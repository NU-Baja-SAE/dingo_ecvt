#include "encoder.h"
#include "config.h"


Encoder::Encoder(gpio_num_t a, gpio_num_t b, pcnt_unit_t counterId) : counterId(counterId)
{
    // Configure PCNT unit

    pcnt_config_t config_a;
    config_a.pulse_gpio_num = a;
    config_a.ctrl_gpio_num = b;
    config_a.hctrl_mode = PCNT_MODE_KEEP; // Rising A on HIGH B = CW Step
    config_a.lctrl_mode = PCNT_MODE_REVERSE; // Rising A on LOW B = CCW Step
    config_a.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge of the a signal
    config_a.neg_mode = PCNT_COUNT_DEC;   // Count down on the negative edge of the a signal
    config_a.counter_h_lim = INT16_MAX;   // Set high limit to maximum 16-bit integer
    config_a.counter_l_lim = INT16_MIN;   // Set low limit to minimum 16-bit integer
    config_a.channel = PCNT_CHANNEL_0;    // Use channel 0 of the PCNT unit
    config_a.unit = counterId;

    pcnt_unit_config(&config_a);

    pcnt_config_t config_b;
    config_b.pulse_gpio_num = b;
    config_b.ctrl_gpio_num = a;
    config_b.hctrl_mode = PCNT_MODE_KEEP; // Rising B on HIGH A = CW Step
    config_b.lctrl_mode = PCNT_MODE_REVERSE; // Rising B on LOW A = CCW Step
    config_b.pos_mode = PCNT_COUNT_DEC;   // Count down on the positive edge of the a signal
    config_b.neg_mode = PCNT_COUNT_INC;   // Count up on the negative edge of the a signal
    config_b.counter_h_lim = INT16_MAX;   // Set high limit to maximum 16-bit integer
    config_b.counter_l_lim = INT16_MIN;   // Set low limit to minimum 16-bit integer
    config_b.channel = PCNT_CHANNEL_1;    // Use channel 1 of the PCNT unit
    config_b.unit = counterId;

    pcnt_unit_config(&config_b);

     // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(counterId, 1000);
    pcnt_filter_enable(counterId);

    // clear and start the counter
    pcnt_counter_clear(counterId);
    pcnt_counter_resume(counterId);
}

int Encoder::getCount()
{
    int16_t raw_count;
    pcnt_get_counter_value(counterId, &raw_count);

    int32_t count = static_cast<int32_t>(raw_count) + this->offset;

    // Keep the hardware counter near zero so long runs do not hit the 16-bit PCNT limit.
    if (raw_count != 0)
    {
        this->offset = count;
        pcnt_counter_clear(counterId);
    }

    this->full_revs = count / COUNT_PER_REV;
    this->last_count = count;
    return count;
}

// returns the encoder position in units of stepper motor steps
int Encoder::getSteps() {
    int32_t count = this->getCount();
    return (count * STEPS_PER_REVOLUTION) / COUNT_PER_REV;
}

void Encoder::resetCount()
{
    pcnt_counter_clear(counterId);
    offset = 0;
    full_revs = 0;
    last_count = 0;
}

void Encoder::setCount(int count)
{
    pcnt_counter_clear(counterId);
    offset = count;
    full_revs = count / COUNT_PER_REV;
    last_count = count;
}