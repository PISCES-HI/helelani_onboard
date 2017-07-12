#ifndef GPSREADER_H
#define GPSREADER_H

#include <thread>
#include <mutex>
#include <gps_common/GPSFix.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/publisher.h>

class GPSReader
{
    std::thread m_thread;
    std::mutex m_lock;
    gps_common::GPSFix m_gpsFix = {};
    sensor_msgs::NavSatFix m_navSatFix = {};
    ros::Publisher m_gpsFixPub;
    ros::Publisher m_navSatFixPub;
    bool m_running = true;
    void PublishReading();
public:
    GPSReader(ros::NodeHandle& n, const std::string& path);
    ~GPSReader();
};

#endif // GPSREADER_H
