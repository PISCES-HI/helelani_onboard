#ifndef GPIODRIVER_H
#define GPIODRIVER_H

class GPIODriver
{
    int m_fds[32];
public:
    GPIODriver(int base);
    ~GPIODriver();
    bool readPin(int pin);
};

#endif // GPIODRIVER_H
