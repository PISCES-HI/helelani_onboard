#include <iostream>
#include <cmath>
#include <unistd.h>

#include "i2c.h"
#include "PwmDriver.h"

// Set to true to print some debug messages, or false to disable them.
const bool ENABLE_DEBUG_OUTPUT = false;

const uint8_t PCA9685_SUBADR1 = 0x2;
const uint8_t PCA9685_SUBADR2 = 0x3;
const uint8_t PCA9685_SUBADR3 = 0x4;

const uint8_t PCA9685_MODE1 = 0x0;
const uint8_t PCA9685_PRESCALE = 0xFE;

const uint8_t LED0_ON_L = 0x6;
const uint8_t LED0_ON_H = 0x7;
const uint8_t LED0_OFF_L = 0x8;
const uint8_t LED0_OFF_H = 0x9;

const uint8_t ALLLED_ON_L = 0xFA;
const uint8_t ALLLED_ON_H = 0xFB;
const uint8_t ALLLED_OFF_L = 0xFC;
const uint8_t ALLLED_OFF_H = 0xFD;

PwmDriver::PwmDriver(I2CInterface& interface) : _handle(interface) {}

void PwmDriver::begin() {
    reset();
}


void PwmDriver::reset() {
    _handle.writeByte(PCA9685_MODE1, 0x0);
}

void PwmDriver::set_pwm_freq(float freq) {
    //Serial.print("Attempting to set freq ");
    //Serial.println(freq);
    freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    if (ENABLE_DEBUG_OUTPUT) {
        std::cout << "Estimated pre-scale: " << prescaleval << std::endl;
    }
    uint8_t prescale = floor(prescaleval + 0.5);
    if (ENABLE_DEBUG_OUTPUT) {
        std::cout << "Final pre-scale: " << prescale << std::endl;
    }

    uint8_t oldmode = _handle.readByte(PCA9685_MODE1);
    uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
    _handle.writeByte(PCA9685_MODE1, newmode); // go to sleep
    _handle.writeByte(PCA9685_PRESCALE, prescale); // set the prescaler
    _handle.writeByte(PCA9685_MODE1, oldmode);
    usleep(5*1000); // Sleep for 5 milliseconds
    _handle.writeByte(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                            // This is why the beginTransmission below was not working.
    // Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void PwmDriver::set_pwm(uint8_t num, uint16_t on, uint16_t off) {
    //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

    uint8_t data[4];
    *((uint16_t*)&data[0]) = on;
    *((uint16_t*)&data[2]) = off;

    _handle.writeBytes(LED0_ON_L+4*num, 4, data);
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void PwmDriver::set_pin(uint8_t num, uint16_t val, bool invert) {
    // Clamp value between 0 and 4095 inclusive.
    val = std::min((uint16_t)val, (uint16_t)4095);
    if (invert) {
        if (val == 0) {
            // Special value for signal fully on.
            set_pwm(num, 4096, 0);
        } else if (val == 4095) {
            // Special value for signal fully off.
            set_pwm(num, 0, 4096);
        } else {
            set_pwm(num, 0, 4095-val);
        }
    } else {
        if (val == 4095) {
            // Special value for signal fully on.
            set_pwm(num, 4096, 0);
        } else if (val == 0) {
            // Special value for signal fully off.
            set_pwm(num, 0, 4096);
        } else {
            set_pwm(num, 0, val);
        }
    }
}
