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
    for (int i=0 ; i<8 ; ++i) {
        // Read pins starting at 24 (to avoid ADC conflicts)
        snprintf(path, 32, "/sys/class/gpio%d/value", base+24+i);
        m_fds[i] = open(path, O_RDONLY);
    }
}

GPIODriver::~GPIODriver()
{
    for (int i=0 ; i<8 ; ++i) {
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
