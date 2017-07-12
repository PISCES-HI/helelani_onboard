#include <ros/console.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "i2c.h"

I2CInterface::I2CInterface(const std::string& path)
{
    if (path.empty())
        return;

    m_fd = open(path.c_str(), O_RDWR);
    if (!m_fd) {
        ROS_ERROR("Unable to open %s: %s", path.c_str(), strerror(errno));
        return;
    }
}

I2CInterface::~I2CInterface()
{
    close(m_fd);
}

int I2CInterface::setAddr(int addr)
{
    if (!m_fd)
        return -EINVAL;
    int ret  = ioctl(m_fd, I2C_SLAVE, addr);
    if (ret < 0)
        ROS_ERROR("Unable to ioctl I2C with address: %d %s", addr, strerror(errno));
    return ret;
}

int I2CInterface::read(unsigned len, uint8_t* buf) const
{
    if (!m_fd)
        return -EINVAL;
    int ret = ::read(m_fd, buf, len);
    if (ret < 0)
        ROS_ERROR("Unable to read I2C: %s", strerror(errno));
    return ret;
}

int I2CInterface::write(unsigned len, const uint8_t* buf) const
{
    if (!m_fd)
        return -EINVAL;
    int ret = ::write(m_fd, buf, len);
    if (ret < 0)
        ROS_ERROR("Unable to write I2C: %s", strerror(errno));
    return ret;
}

int I2CInterface::readByte(uint8_t command) const
{
    if (write(1, &command) < 0)
        return -1;
    uint8_t res;
    if (read(1, &res) < 0)
        return -1;
    return res;
}

int I2CInterface::writeByte(uint8_t command, uint8_t byte) const
{
    uint8_t buf[2] = {command, byte};
    if (write(2, buf) < 0)
        return -1;
    return 0;
}

int I2CInterface::readBytes(uint8_t command, unsigned len, uint8_t* bytes) const
{
    if (write(1, &command) < 0)
        return -1;
    return read(len, bytes);
}

int I2CInterface::writeBytes(uint8_t command, unsigned len, const uint8_t* bytes) const
{
    if (len > 32)
        len = 32;
    uint8_t buf[33];
    buf[0] = command;
    if (len)
        memcpy(buf+1, bytes, len);
    if (write(len+1, buf) < 0)
        return -1;
    return 0;
}

int I2CInterface::readBit(uint8_t command, uint8_t bit) const
{
    int b = readByte(command);
    if (b == -1)
        return -1;
    return b & (1 << bit);
}

int I2CInterface::writeBit(uint8_t command, uint8_t bit, bool value) const
{
    int b = readByte(command);
    if (b == -1)
        return -1;
    b = value ? (b | (1 << bit)) : (b & ~(1 << bit));
    return writeByte(command, b);
}

int I2CInterface::readBits(uint8_t command, uint8_t bit, uint8_t numBits) const
{
    int b = readByte(command);
    if (b == -1)
        return -1;
    uint8_t mask = ((1 << numBits) - 1) << (bit - numBits + 1);
    return (b & mask) >> (bit - numBits + 1);
}

int I2CInterface::writeBits(uint8_t command, uint8_t bit, uint8_t numBits, uint8_t data) const
{
    int b = readByte(command);
    if (b == -1)
        return -1;

    uint8_t mask = ((1 << numBits) - 1) << (bit - numBits + 1);
    data <<= (bit - numBits + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte

    return writeByte(command, b);
}
