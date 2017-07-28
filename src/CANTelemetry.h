#ifndef CANTELEMETRY_H
#define CANTELEMETRY_H

#include <string>
#include <thread>
#include <mutex>

#define GEAR_HI_RATIO (3200.0 / 99.0)
#define GEAR_LO_RATIO (12800.0 / 99.0)
#define GEAR_CHAIN_RATIO (800.0 / 99.0)
#define TWEAK_RATIO 1.25

struct SCANMotorData
{
    int16_t throttle;
    int16_t voltage;
    int16_t current;
    int16_t speed;
    float getCurrent() const { return current / 160.f; }
    float getSpeed(bool hiGear) const
    { return speed * TWEAK_RATIO / (hiGear ? GEAR_HI_RATIO : GEAR_LO_RATIO) / GEAR_CHAIN_RATIO; }
};

class CANMotorData
{
    int m_socket;
    std::thread m_thread;
    bool m_running = true;
    void CommTask(std::string interface,
                  std::function<void(const SCANMotorData&)> updateFunc);
public:
    CANMotorData(const std::string& interface,
                 const std::function<void(const SCANMotorData&)>& updateFunc);
    ~CANMotorData();
};

#endif // CANTELEMETRY_H
