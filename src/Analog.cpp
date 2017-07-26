#include <ros/console.h>
#include <fcntl.h>
#include <signal.h>

#include "Analog.h"

IIOAnalogInterface::IIOAnalogInterface(const std::string& dev,
                                       std::function<void(const uint16_t*)> updateFunc)
: m_updateFunc(updateFunc)
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
            if (read(fd, values, 16) == 16)
                m_updateFunc(values);
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

double get_avionics_temperature(const double analog_reading) {
    // Get voltage (between 0 and 4.5v)
    double voltage = (analog_reading/1023.0) * 4.5;
    // 9.8mv per degree and offset by -30 C
    return ((voltage * 1000.0) / 9.8) - 30.0;
}


double get_ambient_temperature(const double analog_reading) {
    // Get voltage (between 0 and 4.5v)
    double voltage = (analog_reading/1023.0) * 4.5;
    // 9.8mv per degree and offset by -15 C
    return ((voltage * 1000.0) / 9.8) - 15.0;
}
