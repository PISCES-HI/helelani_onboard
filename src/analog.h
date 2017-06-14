#ifndef ANALOG_H
#define ANALOG_H

#include <string>

class IIOAnalogInterface
{
    int m_fds[8];
public:
    IIOAnalogInterface(const std::string& root);
    ~IIOAnalogInterface();
    int GetChannelValue(int chan) const;
};

float read_voltage(const uint16_t analog_reading, const float r1, const float r2);
float get_48v_voltage(const uint16_t analog_reading);
float get_12v_e_voltage(const uint16_t analog_reading);
float get_12v_pl_voltage(const uint16_t analog_reading);

float get_avionics_temperature(const uint16_t analog_reading);
float get_ambient_temperature(const uint16_t analog_reading);

#endif // ANALOG_H
