#include "GPIODriver.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

GPIODriver::GPIODriver(int base)
{
    char path[256];
    for (int i=0 ; i<32 ; ++i) {
        snprintf(path, 32, "/sys/class/gpio%d/value", base+i);
        m_fds[i] = open(path, O_RDONLY);
    }
}

GPIODriver::~GPIODriver()
{
    for (int i=0 ; i<32 ; ++i) {
        close(m_fds[i]);
    }
}

bool GPIODriver::readPin(int pin)
{
    char buf[4];
    if (m_fds[pin]) {
        if (read(m_fds[pin], buf, 4) > 0)
            return atoi(buf) != 0;
    }
    return false;
}
