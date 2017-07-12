#include <ros/console.h>
#include <fcntl.h>
#include <signal.h>

#include "Analog.h"

IIOAnalogInterface::IIOAnalogInterface(const std::string& dev)
{
    if (dev.empty())
        return;
    m_thread = std::thread([this, &dev]()
    {
        int fd = open(dev.c_str(), O_RDONLY);
        if (!fd) {
            ROS_ERROR("Unable to open %s", dev.c_str());
            return;
        }

        while (m_running) {
            uint16_t values[8];
            if (read(fd, values, 16) == 16) {
                std::lock_guard<std::mutex> lk(m_mutex);
                memcpy(m_values, values, 16);
            }
        }

        close(fd);
    });
}

IIOAnalogInterface::~IIOAnalogInterface()
{
    m_running = false;
    if (m_thread.joinable())
    {
        pthread_kill(m_thread.native_handle(), SIGUSR1);
        m_thread.join();
    }
}

uint16_t IIOAnalogInterface::GetChannelValue(int chan)
{
    if (chan > 7)
        return 0;
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_values[chan];
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
