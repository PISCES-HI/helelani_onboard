#include <ros/console.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "i2c.h"

I2CInterface::I2CInterface(int fd, int addr)
: m_fd(fd)
{
    if (ioctl(m_fd, I2C_SLAVE, addr) < 0)
        ROS_ERROR("Unable to ioctl I2C with address %d %s", addr, strerror(errno));
}

I2CInterface::~I2CInterface()
{
    close(m_fd);
}

int I2CInterface::read(char* buf, int len) const
{
    int ret = ::read(m_fd, buf, len);
    if (ret < 0) {
        ROS_ERROR("Unable to read I2C %s", strerror(errno));
        return 0;
    }
    return ret;
}

int I2CInterface::write(const char* buf, int len) const
{
    int ret = ::write(m_fd, buf, len);
    if (ret < 0) {
        ROS_ERROR("Unable to write I2C %s", strerror(errno));
        return 0;
    }
    return ret;
}

int I2CInterface::readSmbusByte(uint8_t command) const
{
    __s32 res = i2c_smbus_read_byte_data(m_fd, command);
    if (res == -1) {
        ROS_ERROR("Unable to read byte with I2C %s", strerror(errno));
        return -1;
    }
    return res;
}

int I2CInterface::writeSmbusByte(uint8_t command, uint8_t byte) const
{
    if (i2c_smbus_write_byte_data(m_fd, command, byte)) {
        ROS_ERROR("Unable to write byte with I2C %s", strerror(errno));
        return -1;
    }
    return 0;
}

int I2CInterface::readSmbusBytes(uint8_t command, int len, uint8_t* bytes) const
{
    if (i2c_smbus_read_block_data(m_fd, command, bytes)) {
        ROS_ERROR("Unable to read bytes with I2C %s", strerror(errno));
        return -1;
    }
    return 0;
}

int I2CInterface::writeSmbusBytes(uint8_t command, int len, const uint8_t* bytes) const
{
    if (i2c_smbus_write_block_data(m_fd, command, len, bytes)) {
        ROS_ERROR("Unable to write bytes with I2C %s", strerror(errno));
        return -1;
    }
    return 0;
}

int I2CInterface::readSmbusBit(uint8_t command, uint8_t bit) const
{
    int b = readSmbusByte(command);
    if (b == -1)
        return -1;
    return b & (1 << bit);
}

int I2CInterface::writeSmbusBit(uint8_t command, uint8_t bit, bool value) const
{
    int b = readSmbusByte(command);
    if (b == -1)
        return -1;
    b = value ? (b | (1 << bit)) : (b & ~(1 << bit));
    return writeSmbusByte(command, b);
}

int I2CInterface::readSmbusBits(uint8_t command, uint8_t bit, uint8_t numBits) const
{
    int b = readSmbusByte(command);
    if (b == -1)
        return -1;
    uint8_t mask = ((1 << numBits) - 1) << (bit - numBits + 1);
    return (b & mask) >> (bit - numBits + 1);
}

int I2CInterface::writeSmbusBits(uint8_t command, uint8_t bit, uint8_t numBits, uint8_t data) const
{
    int b = readSmbusByte(command);
    if (b == -1)
        return -1;

    uint8_t mask = ((1 << numBits) - 1) << (bit - numBits + 1);
    data <<= (bit - numBits + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    return writeSmbusByte(command, b);
}
