#include <ros/ros.h>
#include "DlnFinders.h"
#include "PwmDriver.h"
#include "StereoCameraCapture.h"
#include <std_msgs/builtin_string.h>
#include <helelani_common/CameraCtrl.h>
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

class ServoController
{
    PwmDriver m_pwm;
    ros::Subscriber m_situationSub;
    ros::Subscriber m_stereoSub;
    StereoCameraService& m_stereoService;
public:
    ServoController(ros::NodeHandle& n,
                    I2CInterface& pwmInterface,
                    StereoCameraService& stereoSrv)
    : m_pwm(pwmInterface),
      m_situationSub(n.subscribe("/helelani/situation_cam_ctrl", 1000,
                                 &ServoController::updateSituation, this)),
      m_stereoSub(n.subscribe("/helelani/stereo_cam_ctrl", 1000,
                              &ServoController::updateStereo, this)),
      m_stereoService(stereoSrv)
    {
        m_pwm.begin();
        m_pwm.set_pwm_freq(50);
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
