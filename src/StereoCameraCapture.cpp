#include "StereoCameraCapture.h"
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>

StereoCameraCapture::StereoCameraCapture(const std::string& path,
                                         ros::NodeHandle& n,
                                         const std::string& topic)
: m_topic(topic)
{
    unsigned int i;
    int ret = 0;

    if ((m_fd = open(path.c_str(), O_RDWR)) == -1) {
        ROS_ERROR("ERROR opening V4L interface");
        return;
    }
    memset(&m_cap, 0, sizeof(struct v4l2_capability));
    ret = ioctl(m_fd, VIDIOC_QUERYCAP, &m_cap);
    if (ret < 0) {
        ROS_ERROR("Error opening device %s: unable to query device.",
                  path.c_str());
        goto fatal;
    }

    if ((m_cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
        ROS_ERROR("Error opening device %s: video capture not supported.",
                  path.c_str());
        goto fatal;
    }
    if (!(m_cap.capabilities & V4L2_CAP_STREAMING)) {
        ROS_ERROR("%s does not support streaming i/o",
                  path.c_str());
        goto fatal;
    }

    /* set format in */
    memset(&m_fmt, 0, sizeof(struct v4l2_format));
    m_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m_fmt.fmt.pix.width = CAM_WIDTH;
    m_fmt.fmt.pix.height = CAM_HEIGHT;
    m_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    m_fmt.fmt.pix.field = V4L2_FIELD_ANY;
    ret = ioctl(m_fd, VIDIOC_S_FMT, &m_fmt);
    if (ret < 0) {
        ROS_ERROR("Unable to set format: %d.", errno);
        goto fatal;
    }
    if ((m_fmt.fmt.pix.width != CAM_WIDTH) ||
        (m_fmt.fmt.pix.height != CAM_HEIGHT)) {
        ROS_ERROR("format asked unavailable get width %d height %d",
                  m_fmt.fmt.pix.width, m_fmt.fmt.pix.height);
        goto fatal;
    }

    /* set stream parameters */
    memset(&m_sparm, 0, sizeof(struct v4l2_streamparm));
    m_sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(m_fd, VIDIOC_G_PARM, &m_sparm);
    if (ret < 0) {
        ROS_ERROR("Unable to get stream parameters: %d.", errno);
        goto fatal;
    }

    m_sparm.parm.capture.timeperframe.numerator = 1;
    m_sparm.parm.capture.timeperframe.denominator = 5;
    ret = ioctl(m_fd, VIDIOC_S_PARM, &m_sparm);
    if (ret < 0) {
        ROS_ERROR("Unable to set stream parameters: %d.", errno);
        goto fatal;
    }

    /* request buffers */
    memset(&m_rb, 0, sizeof(struct v4l2_requestbuffers));
    m_rb.count = NB_CAM_BUFFER;
    m_rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m_rb.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(m_fd, VIDIOC_REQBUFS, &m_rb);
    if (ret < 0) {
        ROS_ERROR("Unable to allocate buffers: %d.", errno);
        goto fatal;
    }
    /* map the buffers */
    for (i = 0; i < NB_CAM_BUFFER; i++) {
        memset(&m_buf, 0, sizeof(struct v4l2_buffer));
        m_buf.index = i;
        m_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        m_buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(m_fd, VIDIOC_QUERYBUF, &m_buf);
        if (ret < 0) {
            ROS_ERROR("Unable to query buffer (%d).", errno);
            goto fatal;
        }
        //if (m_buf.length % pageSize)
        //    m_buf.length = (m_buf.length / pageSize + 1) * pageSize;
        ROS_DEBUG("length: %u offset: %u", m_buf.length,
                  m_buf.m.offset);
        m_mem[i] = mmap(0 /* start anywhere */ ,
                        m_buf.length, PROT_READ, MAP_SHARED, m_fd,
                        m_buf.m.offset);
        if (m_mem[i] == MAP_FAILED) {
            ROS_ERROR("Unable to map buffer (%d)", errno);
            goto fatal;
        }
        ROS_DEBUG("Buffer mapped at address %p.", m_mem[i]);
    }

    m_framebuffer.reset(new uint8_t[CAM_FRAMESIZE]);
    memset(m_framebuffer.get(), 0, CAM_FRAMESIZE);
    m_img.height = CAM_HEIGHT;
    m_img.width = CAM_WIDTH;
    m_img.encoding = sensor_msgs::image_encodings::MONO8;
    m_img.is_bigendian = 0;
    m_img.step = CAM_WIDTH;
    m_img.data.resize(CAM_FRAMESIZE);

    m_imgPublisher = n.advertise<sensor_msgs::Image>(topic, 10);

    return;
fatal:
    close(m_fd);
    m_fd = 0;
}

StereoCameraCapture::~StereoCameraCapture()
{
    /* If the memory maps are not released the device will remain opened even
     * after a call to close(); */
    for(int i = 0; i < NB_CAM_BUFFER; i++)
        munmap(m_mem[i], m_buf.length);

    close(m_fd);
}

int StereoCameraCapture::enable()
{
    if (!m_fd)
        return -1;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(m_fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        ROS_ERROR("Unable to %s capture: %d.", "start", errno);
        return ret;
    }
    return 0;
}

int StereoCameraCapture::disable()
{
    if (!m_fd)
        return -1;

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(m_fd, VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
        ROS_ERROR("Unable to %s capture: %d.", "stop", errno);
        return ret;
    }
    return 0;
}

bool StereoCameraCapture::grab(uint8_t* dataOut)
{
    if (!m_fd)
        return false;

    int ret;
    memset(&m_buf, 0, sizeof(struct v4l2_buffer));
    m_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m_buf.memory = V4L2_MEMORY_MMAP;
    m_buf.index = 0;
    m_buf.bytesused = 0;

    enable();

    for (int i=0 ; i<2 || m_buf.bytesused != CAM_FRAMESIZE ; ++i)
    {
        ret = ioctl(m_fd, VIDIOC_QBUF, &m_buf);
        if (ret < 0) {
            ROS_ERROR("Unable to queue buffer (%d).", errno);
            return false;
        }

        ret = ioctl(m_fd, VIDIOC_DQBUF, &m_buf);
        if (ret < 0) {
            ROS_ERROR("Unable to dequeue buffer (%d).", errno);
            return false;
        }
        //ROS_INFO("%s %d %d\n", m_topic.c_str(), m_buf.index, int(m_buf.bytesused));

        if (m_buf.bytesused == CAM_FRAMESIZE)
            memmove(dataOut, m_mem[m_buf.index], CAM_FRAMESIZE);
    }

    disable();

    return true;
}

void StereoCameraCapture::publish()
{
    if (grab(m_img.data.data()))
    {
        m_img.header.stamp = ros::Time::now();
        m_imgPublisher.publish(m_img);
    }
}

void StereoCameraCapture::changeExposure(int exposure)
{
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = exposure / 5 + 1;
    ioctl(m_fd, VIDIOC_S_CTRL, &ctrl);
}

bool StereoCameraService::captureService(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& resp)
{
    m_cv.notify_one();
    return true;
}

StereoCameraService::StereoCameraService(ros::NodeHandle& n)
: m_left("/dev/videoStereoLeft", n, "/helelani/stereo_image_left"),
  m_right("/dev/videoStereoRight", n, "/helelani/stereo_image_right"),
  m_srv(n.advertiseService("/helelani/stereo_image_request",
        &StereoCameraService::captureService, this))
{
    m_thread = std::thread([this]()
    {
        while (true)
        {
            std::unique_lock<std::mutex> lk(m_mutex);
            m_cv.wait(lk);
            if (!m_running)
                break;
            if (m_exposureChange)
            {
                m_exposureChange = false;
                m_left.changeExposure(m_exposure);
                m_right.changeExposure(m_exposure);
            }
            m_left.publish();
            m_right.publish();
        }
    });
}

StereoCameraService::~StereoCameraService()
{
    if (m_thread.joinable())
    {
        m_running = false;
        pthread_kill(m_thread.native_handle(), SIGUSR1);
        m_cv.notify_one();
        m_thread.join();
    }
}