#ifndef ANALOG_H
#define ANALOG_H

#include <string>
#include <thread>
#include <mutex>

class IIOAnalogInterface
{
    std::function<void(const uint16_t*)> m_updateFunc;
    std::thread m_thread;
    bool m_running = true;
public:
    IIOAnalogInterface(const std::string& dev,
                       std::function<void(const uint16_t*)> updateFunc);
    ~IIOAnalogInterface();
};

double get_avionics_temperature(const double analog_reading);
double get_ambient_temperature(const double analog_reading);

#endif // ANALOG_H
