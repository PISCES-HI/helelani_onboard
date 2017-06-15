#ifndef GPSREADER_H
#define GPSREADER_H

#include <thread>
#include <mutex>
#include <gps_common/GPSFix.h>
#include <ros/publisher.h>

class GPSReader
{
    std::thread m_thread;
    std::mutex m_lock;
    gps_common::GPSFix m_reading = {};
    ros::Publisher m_gpsPub;
    bool m_running = true;
    bool m_dataReady = false;
public:
    GPSReader(ros::NodeHandle& n, const std::string& path);
    ~GPSReader();
    void PublishReading();
};

#endif // GPSREADER_H
