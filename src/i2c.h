#ifndef I2C_H
#define I2C_H

#include <stdint.h>

const unsigned int PWM_I2C_ADDR = 0x40;

class I2CInterface
{
    int m_fd;
public:
    I2CInterface(int fd, int addr);
    ~I2CInterface();
    int read(char* buf, int len) const;
    int write(const char* buf, int len) const;
    int readSmbusByte(uint8_t command) const;
    int writeSmbusByte(uint8_t command, uint8_t byte) const;
    int readSmbusBytes(uint8_t command, int len, uint8_t* bytes) const;
    int writeSmbusBytes(uint8_t command, int len, const uint8_t* bytes) const;
    int readSmbusBit(uint8_t command, uint8_t bit) const;
    int writeSmbusBit(uint8_t command, uint8_t bit, bool value) const;
    int readSmbusBits(uint8_t command, uint8_t bit, uint8_t numBits) const;
    int writeSmbusBits(uint8_t command, uint8_t bit, uint8_t numBits, uint8_t data) const;
};

#endif // I2C_H
