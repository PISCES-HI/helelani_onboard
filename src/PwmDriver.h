#ifndef PWMDRIVER_H
#define PWMDRIVER_H

#include "i2c.h"
#define PWM_ADDR 0x40

class PwmDriver {
    public:
        PwmDriver(I2CInterface& interface);

        void begin();
        void reset();
        void set_pwm_freq(float freq);
        void set_pwm(uint8_t num, uint16_t on, uint16_t off);
        void set_pin(uint8_t num, uint16_t val, bool invert=false);

    private:
        I2CInterface& _handle;
};

#endif // PWMDRIVER_H
