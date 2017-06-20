#ifndef CANTELEMETRY_H
#define CANTELEMETRY_H

#include <string>
#include <thread>
#include <mutex>

#define GEAR_HI_RATIO (3200.0 / 99.0)
#define GEAR_LO_RATIO (12800.0 / 99.0)
#define GEAR_CHAIN_RATIO (800.0 / 99.0)

struct SCANMotorData
{
    int16_t throttle;
    int16_t voltage;
    int16_t current;
    int16_t speed;
    float getCurrent() const { return current / 160.f; }
    float getSpeed(bool hiGear) const
    { return speed / (hiGear ? GEAR_HI_RATIO : GEAR_LO_RATIO) / GEAR_CHAIN_RATIO; }
};

class CANMotorData
{
    int m_socket;
    std::thread m_thread;
    std::mutex m_mutex;
    SCANMotorData m_data;
    bool m_running = true;
    void CommTask(std::string interface);
public:
    CANMotorData(const std::string& interface);
    ~CANMotorData();
    SCANMotorData getData();
};

#endif // CANTELEMETRY_H
