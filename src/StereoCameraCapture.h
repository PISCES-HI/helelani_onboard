#ifndef STEREOCAMERACAPTURE_H
#define STEREOCAMERACAPTURE_H

#include <linux/videodev2.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <mutex>
#include <condition_variable>
#include <thread>

#define CAM_WIDTH 2592
#define CAM_HEIGHT 1944
#define CAM_FRAMESIZE (CAM_WIDTH * CAM_HEIGHT)
#define NB_CAM_BUFFER 1

class StereoCameraCapture
{
    int m_fd;
    struct v4l2_capability m_cap;
    struct v4l2_format m_fmt;
    struct v4l2_streamparm m_sparm;
    struct v4l2_buffer m_buf;
    struct v4l2_requestbuffers m_rb;
    void* m_mem[NB_CAM_BUFFER];
    std::unique_ptr<uint8_t[]> m_framebuffer;

    std::string m_topic;
    sensor_msgs::Image m_img;
    ros::Publisher m_imgPublisher;

    int enable();
    int disable();
public:
    StereoCameraCapture(const std::string& path, ros::NodeHandle& n,
                        const std::string& topic);
    ~StereoCameraCapture();

    bool grab(uint8_t* dataOut);
    void publish();
};

class StereoCameraService
{
    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::thread m_thread;
    StereoCameraCapture m_left;
    StereoCameraCapture m_right;
    ros::ServiceServer m_srv;
    bool m_running = true;
    bool captureService(std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& resp);
public:
    StereoCameraService(ros::NodeHandle& n);
    ~StereoCameraService();
};

#endif // STEREOCAMERACAPTURE_H
