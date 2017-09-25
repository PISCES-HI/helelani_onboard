#include <ros/ros.h>
#include "DlnFinders.h"
#include "PwmDriver.h"
#include "StereoCameraCapture.h"
#include <std_msgs/builtin_string.h>
#include <std_msgs/Float32.h>
#include <helelani_common/CameraCtrl.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>

static void SigUsrHandler(int) {}

static double ServoMap(double val, double min_a, double max_a,
                       double min_b, double max_b) {
    val = std::max(std::min(val, max_a), min_a);
    return ((val - min_a)/(max_a - min_a))*(max_b-min_b) + min_b;
}

const int PWM_SERVO_MIN = 150;
const int PWM_SERVO_MAX = 450;
const int PWM_NEW_SERVO_MIN = 150;
const int PWM_NEW_SERVO_MAX = 500;

static const tf::Quaternion g_lidarLocalRot({0.f, 0.f, 1.f}, tfRadians(135.f));

class ServoController
{
    PwmDriver m_pwm;
    ros::Subscriber m_situationSub;
    ros::Subscriber m_stereoSub;
    ros::Subscriber m_lidarSub;
    StereoCameraService& m_stereoService;
    tf::TransformBroadcaster m_tfBr;
    ros::Timer m_tfTimer;
public:
    ServoController(ros::NodeHandle& n,
                    I2CInterface& pwmInterface,
                    StereoCameraService& stereoSrv)
    : m_pwm(pwmInterface),
      m_situationSub(n.subscribe("/helelani/situation_cam_ctrl", 1000,
                                 &ServoController::updateSituation, this)),
      m_stereoSub(n.subscribe("/helelani/stereo_cam_ctrl", 1000,
                              &ServoController::updateStereo, this)),
      m_lidarSub(n.subscribe("/helelani/lidar_ctrl", 1000,
                             &ServoController::updateLidar, this)),
      m_stereoService(stereoSrv),
      m_tfTimer(n.createTimer(ros::Rate(30), &ServoController::updateLidarFrame, this))
    {
        m_pwm.begin();
        m_pwm.set_pwm_freq(50);
        std_msgs::Float32 msg;
        msg.data = 82.f;
        updateLidar(msg);
    }

    void updateSituation(const helelani_common::CameraCtrl& message)
    {
        uint16_t duty_cycle;
        duty_cycle = uint16_t(ServoMap(message.pan, 0.0, 180.0,
                                       PWM_SERVO_MIN, PWM_SERVO_MAX));
        m_pwm.set_pin(1, duty_cycle);
        duty_cycle = uint16_t(ServoMap(std::max(45.f, -message.tilt + 180.f + 15.f), 0.0, 180.0,
                                       PWM_NEW_SERVO_MIN, PWM_NEW_SERVO_MAX));
        m_pwm.set_pin(2, duty_cycle);
    }

    void updateStereo(const helelani_common::CameraCtrl& message)
    {
        uint16_t duty_cycle;
        duty_cycle = uint16_t(ServoMap(-message.pan + 180.f, 0.0, 180.0,
                                       PWM_NEW_SERVO_MIN, PWM_NEW_SERVO_MAX));
        m_pwm.set_pin(3, duty_cycle);
        duty_cycle = uint16_t(ServoMap(message.tilt, 0.0, 180.0,
                                       PWM_NEW_SERVO_MIN, PWM_NEW_SERVO_MAX));
        m_pwm.set_pin(4, duty_cycle);

        m_stereoService.changeExposure(message.exposure);
    }

    tfScalar m_targetLidarAngle = 0.f;
    tfScalar m_lidarAngle = 0.f;
    tf::Quaternion m_lidarQuat = tf::Quaternion::getIdentity();

    void updateLidar(const std_msgs::Float32& message)
    {
        m_targetLidarAngle = message.data - 82.f;
        uint16_t duty_cycle;
        duty_cycle = uint16_t(ServoMap(message.data, 0.0, 180.0,
                                       PWM_NEW_SERVO_MIN, PWM_NEW_SERVO_MAX));
        m_pwm.set_pin(0, duty_cycle);
        updateLidarFrame({});
    }

    void updateLidarFrame(const ros::TimerEvent& te)
    {
        if (m_targetLidarAngle < m_lidarAngle)
        {
            m_lidarAngle -= (te.current_real - te.last_real).toSec() * 180.f;
            if (m_targetLidarAngle > m_lidarAngle)
                m_lidarAngle = m_targetLidarAngle;
            m_lidarQuat = tf::Quaternion({0.f, -1.f, 0.f}, tfRadians(m_lidarAngle));
        }
        else if (m_targetLidarAngle > m_lidarAngle)
        {
            m_lidarAngle += (te.current_real - te.last_real).toSec() * 180.f;
            if (m_targetLidarAngle < m_lidarAngle)
                m_lidarAngle = m_targetLidarAngle;
            m_lidarQuat = tf::Quaternion({0.f, -1.f, 0.f}, tfRadians(m_lidarAngle));
        }
        tf::Transform xf;
        xf.setRotation(m_lidarQuat * g_lidarLocalRot);
        m_tfBr.sendTransform(tf::StampedTransform(xf, ros::Time::now(), "base_link", "laser_frame"));
    }
};

int main(int argc, char *argv[])
{
    // Register dummy SIGUSR1 for terminating threads
    struct sigaction sa = {};
    sa.sa_handler = SigUsrHandler;
    sigaction(SIGUSR1, &sa, nullptr);

    // Setup ROS
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;

    // Find upper DLN
    std::string upper_i2c_path;
    find_upper_dln(upper_i2c_path);

    // Start stereo capture thread and advertise service
    StereoCameraService stereoService(n);

    // Start servo controller
    I2CInterface pwmI2C(upper_i2c_path, PWM_ADDR);
    ServoController servoController(n, pwmI2C, stereoService);

    // Begin update loop
    ros::spin();

    return 0;
}
