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

        ssize_t nbytes = read(m_socket, &frame, sizeof(struct can_frame));

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
            updateFunc(*reinterpret_cast<const SCANMotorData*>(frame.data));
        }
    }

    close(m_socket);
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
