#include "CANTelemetry.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>

void CANMotorData::CommTask(std::string interface,
                            std::function<void(const SCANMotorData&)> updateFunc)
{
    auto doBind = [&interface]()
    {
        struct sockaddr_can addr;
        struct ifreq ifr;

        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock == -1)
            return -1;

        strcpy(ifr.ifr_name, interface.c_str());
        if (ioctl(sock, SIOCGIFINDEX, &ifr))
        {
            ROS_ERROR("Unable to SIOCGIFINDEX %s: %s\n", ifr.ifr_name, strerror(errno));
            close(sock);
            return -1;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)))
        {
            ROS_ERROR("Unable to bind %s: %s\n", ifr.ifr_name, strerror(errno));
            close(sock);
            return -1;
        }
        else
        {
            ROS_INFO("Bound %s\n", ifr.ifr_name);
            return sock;
        }
    };
    int sock = doBind();

    while (m_running)
    {
        struct can_frame frame;

        ssize_t nbytes = -1;
        if (sock != -1)
            nbytes = read(sock, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            ROS_ERROR("%d can raw socket %s read error %s\n", sock, interface.c_str(), strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            if (sock != -1)
                close(sock);
            sock = doBind();
            continue;
        }

        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
            ROS_ERROR("read %s: incomplete CAN frame", interface.c_str());
            continue;
        }

        if (frame.can_id == 0x290) {
            updateFunc(*reinterpret_cast<const SCANMotorData*>(frame.data));
        }
    }

    if (sock != -1)
        close(sock);
}

CANMotorData::CANMotorData(const std::string& interface,
                           const std::function<void(const SCANMotorData&)>& updateFunc)
{
    m_thread = std::thread(std::bind(&CANMotorData::CommTask, this,
                                     interface, updateFunc));
}

CANMotorData::~CANMotorData()
{
    m_running = false;
    if (m_thread.joinable())
    {
        pthread_kill(m_thread.native_handle(), SIGUSR1);
        m_thread.join();
    }
}
