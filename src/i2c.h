#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <string>

const unsigned int PWM_I2C_ADDR = 0x40;

class I2CInterface
{
    int m_fd;
public:
    I2CInterface(const std::string& path, int addr);
    ~I2CInterface();
    operator bool() const { return m_fd != 0; }
    int read(unsigned len, uint8_t* buf) const;
    int write(unsigned len, const uint8_t* buf) const;
    int readByte(uint8_t command) const;
    int writeByte(uint8_t command, uint8_t byte) const;
    int readBytes(uint8_t command, unsigned len, uint8_t* bytes) const;
    int writeBytes(uint8_t command, unsigned len, const uint8_t* bytes) const;
    int readBit(uint8_t command, uint8_t bit) const;
    int writeBit(uint8_t command, uint8_t bit, bool value) const;
    int readBits(uint8_t command, uint8_t bit, uint8_t numBits) const;
    int writeBits(uint8_t command, uint8_t bit, uint8_t numBits, uint8_t data) const;
};

#endif // I2C_H
