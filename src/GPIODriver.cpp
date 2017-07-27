#include "GPIODriver.h"
#include <stdlib.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

GPIODriver::GPIODriver(int base)
: m_base(base)
{}

bool GPIODriver::readPin(int pin) const
{
    char path[32];
    snprintf(path, 32, "/sys/class/gpio/gpio%d/value", m_base + 24 + pin);
    if (int fd = open(path, O_RDONLY))
    {
        char buf[4];
        ssize_t rdSz = read(fd, buf, 4);
        close(fd);
        if (rdSz > 0)
            return buf[0] != '0';
    }
    return false;
}
