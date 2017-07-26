#ifndef GPIODRIVER_H
#define GPIODRIVER_H

class GPIODriver
{
    int m_base;
public:
    explicit GPIODriver(int base);
    bool readPin(int pin) const;
};

#endif // GPIODRIVER_H
