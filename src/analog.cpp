#include <ros/console.h>
#include <fcntl.h>

#include "analog.h"

IIOAnalogInterface::IIOAnalogInterface(const std::string& root)
{
    char path_buf[512];
    for (int i=0 ; i<8 ; ++i) {
        snprintf(path_buf, 512, "%s/in_voltage%d_raw", root.c_str(), i);
        m_fds[i] = open(path_buf, O_RDONLY);
    }
}

IIOAnalogInterface::~IIOAnalogInterface()
{
    for (int i=0 ; i<8 ; ++i)
        close(m_fds[i]);
}

int IIOAnalogInterface::GetChannelValue(int chan) const
{
    if (chan > 7)
        return 0;
    char buf[16];
    ssize_t bytes_read = read(m_fds[chan], buf, 16);
    if (bytes_read <= 0)
        return 0;
    return atoi(buf);
}

float read_voltage(const uint16_t analog_reading, const float r1, const float r2) {
    float scaled_reading = (((float)analog_reading) / 1024.f) * 5.f; // Normalize and scale by Vcc (5v)

    return (scaled_reading / r2) * (r1 + r2);
}

float get_48v_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 17868.0, 950.0);
}

float get_12v_e_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 19890.0, 1899.0);
}

float get_12v_pl_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 19890.0, 1899.0);
}

float get_avionics_temperature(const uint16_t analog_reading) {
    // Get voltage (between 0 and 4.5v)
    float voltage = (float(analog_reading)/1023.0) * 4.5;
    // 9.8mv per degree and offset by -30 C
    return ((voltage * 1000.0) / 9.8) - 30.0;
}


float get_ambient_temperature(const uint16_t analog_reading) {
    // Get voltage (between 0 and 4.5v)
    float voltage = (float(analog_reading)/1023.0) * 4.5;
    // 9.8mv per degree and offset by -15 C
    return ((voltage * 1000.0) / 9.8) - 15.0;
}
