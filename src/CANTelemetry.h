#ifndef CANTELEMETRY_H
#define CANTELEMETRY_H

#include <string>
#include <thread>
#include <mutex>

struct SCANMotorData
{
    int16_t throttle;
    int16_t voltage;
    int16_t current;
    int16_t speed;
};

class CANConnection
{
    int m_socket;
    std::thread m_thread;
    std::mutex m_mutex;
    SCANMotorData m_data;
    bool m_running = true;
    void CommTask();
protected:
    SCANMotorData getData() const;
public:
    CANConnection(const std::string& interface);
    ~CANConnection();
};

class CANMotorData : public CANConnection
{
public:
    CANMotorData(const std::string& interface);
    float getCurrent() const;
};

void FakeMain()
{
    CANMotorData left("canLeft");
    printf("LEFT CURRENT: %f\n", left.getCurrent());

    CANMotorData right("canRight");
    printf("RIGHT CURRENT: %f\n", right.getCurrent());
}

#endif // CANTELEMETRY_H
