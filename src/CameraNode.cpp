#include <ros/ros.h>
#include "DlnFinders.h"
#include "PwmDriver.h"
#include <std_msgs/builtin_string.h>
#include <helelani_common/CameraCtrl.h>

static double ServoMap(double val, double min_a, double max_a, double min_b, double max_b) {
    return ((val - min_a)/(max_a - min_a))*(max_b-min_b) + min_b;
}

const int PWM_SERVO_MIN = 150;
const int PWM_SERVO_MAX = 450;

class ServoController
{
    PwmDriver m_pwm;
    ros::Subscriber m_ptzSub;
public:
    ServoController(ros::NodeHandle& n,
                    I2CInterface& pwmInterface)
    : m_pwm(pwmInterface),
      m_ptzSub(n.subscribe("/helelani/cam_ctrl", 1000, &ServoController::updatePTZ, this))
    {
        m_pwm.begin();
        m_pwm.set_pwm_freq(50);
    }
    void updatePTZ(const helelani_common::CameraCtrl& message)
    {
        int duty_cycle;
        duty_cycle = ServoMap(message.lidar_tilt, 0.0, 180.0,
                              PWM_SERVO_MIN, PWM_SERVO_MAX);
        m_pwm.set_pin(0, duty_cycle);
        duty_cycle = ServoMap(message.situation_pan, 0.0, 180.0,
                              PWM_SERVO_MIN, PWM_SERVO_MAX);
        m_pwm.set_pin(1, duty_cycle);
        duty_cycle = ServoMap(message.situation_tilt, 0.0, 180.0,
                              PWM_SERVO_MIN, PWM_SERVO_MAX);
        m_pwm.set_pin(2, duty_cycle);
        duty_cycle = ServoMap(message.stereo_pan, 0.0, 180.0,
                              PWM_SERVO_MIN, PWM_SERVO_MAX);
        m_pwm.set_pin(3, duty_cycle);
        duty_cycle = ServoMap(message.stereo_tilt, 0.0, 180.0,
                              PWM_SERVO_MIN, PWM_SERVO_MAX);
        m_pwm.set_pin(4, duty_cycle);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;

    // Find upper DLN
    std::string upper_i2c_path;
    find_upper_dln(upper_i2c_path);

    I2CInterface pwmI2C(upper_i2c_path, PWM_ADDR);
    ServoController servoController(n, pwmI2C);

    // Begin update loop
    ros::Rate r(50); // 50 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
