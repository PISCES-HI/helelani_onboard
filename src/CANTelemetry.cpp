#include "CANTelemetry.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <ros/ros.h>
#include <unistd.h>
#include <signal.h>

void CANMotorData::CommTask(std::string interface)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, interface.c_str());
    ioctl(m_socket, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_socket, (struct sockaddr *)&addr, sizeof(addr))) {
        fprintf(stderr, "Unable to bind %s\n", strerror(errno));
        m_running = false;
    }

    while (m_running)
    {
        struct can_frame frame;

        int nbytes = read(m_socket, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            ROS_ERROR("can raw socket read\n");
            continue;
        }

        /* paranoid check ... */
        if (nbytes < sizeof(struct can_frame)) {
            ROS_ERROR("read: incomplete CAN frame");
            continue;
        }

        if (frame.can_id == 0x290) {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_data = *reinterpret_cast<const SCANMotorData*>(frame.data);
        }
    }

    close(m_socket);
}

SCANMotorData CANMotorData::getData()
{
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_data;
}

CANMotorData::CANMotorData(const std::string& interface)
{
    m_thread = std::thread(std::bind(&CANMotorData::CommTask, this, interface));
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
