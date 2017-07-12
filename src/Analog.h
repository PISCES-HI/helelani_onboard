#ifndef ANALOG_H
#define ANALOG_H

#include <string>
#include <thread>
#include <mutex>

class IIOAnalogInterface
{
    std::thread m_thread;
    std::mutex m_mutex;
    uint16_t m_values[8] = {};
    bool m_running = true;
public:
    IIOAnalogInterface(const std::string& dev);
    ~IIOAnalogInterface();
    uint16_t GetChannelValue(int chan);
};

float read_voltage(const uint16_t analog_reading, const float r1, const float r2);
float get_48v_voltage(const uint16_t analog_reading);
float get_12v_e_voltage(const uint16_t analog_reading);
float get_12v_pl_voltage(const uint16_t analog_reading);

float get_avionics_temperature(const uint16_t analog_reading);
float get_ambient_temperature(const uint16_t analog_reading);

#endif // ANALOG_H
