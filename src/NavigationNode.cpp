#include <ros/ros.h>
#include <helelani_common/DriveCommand.h>
#include <helelani_common/Motor.h>
#include <helelani_common/Imu.h>
#include <helelani_common/Throttle.h>
#include <helelani_common/helelani_common.h>
#include "PwmDriver.h"
#include "DlnFinders.h"

#define PWM_MOTOR_MIN 186
#define PWM_MOTOR_MAX 372
#define PWM_MOTOR_NEUTRAL 279

#define SADL_PIN 0
#define R_MOTOR_PIN 1
#define L_MOTOR_PIN 2
#define BRAKE_PIN 3

#define DEFAULT_THROTTLE 50.f

static float ServoMap(float val, float min_a, float max_a,
                      float min_b, float max_b) {
    val = std::max(std::min(val, max_a), min_a);
    return ((val - min_a)/(max_a - min_a))*(max_b-min_b) + min_b;
}

class NavigationDriver;

class ICommand
{
public:
    virtual ~ICommand() = default;
    virtual bool update(NavigationDriver& driver, double dt) = 0;
};

class NavigationDriver
{
    PwmDriver m_pwm;
    ros::NodeHandle& m_nh;
    std::list<std::unique_ptr<ICommand>> m_cmds;
    ros::Subscriber m_cmdSub;
    ros::Subscriber m_throttleSub;
    ros::Subscriber m_leftMotorSub;
    ros::Subscriber m_rightMotorSub;
    ros::Subscriber m_imuSub;
    float m_leftRotations = 0.f;
    float m_rightRotations = 0.f;
    double m_imuPitch = 0.0;
    double m_imuYaw = 0.0;
    float m_cachedThrottle = DEFAULT_THROTTLE;
    void _cmdCallback(const helelani_common::DriveCommand& cmd);
    void _throttleCallback(const helelani_common::Throttle& cmd);

    void _leftMotorCallback(const helelani_common::Motor& msg)
    {
        m_leftRotations = msg.rotations;
    }
    void _rightMotorCallback(const helelani_common::Motor& msg)
    {
        m_rightRotations = msg.rotations;
    }

    void _imuCallback(const helelani_common::Imu& msg)
    {
        m_imuPitch = msg.pitch;
        m_imuYaw = msg.yaw;
    }

public:
    NavigationDriver(I2CInterface& pwmI2C, ros::NodeHandle& n)
    : m_pwm(pwmI2C),
      m_nh(n),
      m_cmdSub(n.subscribe("/helelani/drive_cmd", 10, &NavigationDriver::_cmdCallback, this)),
      m_throttleSub(n.subscribe("/helelani/throttle", 10, &NavigationDriver::_throttleCallback, this)),
      m_leftMotorSub(n.subscribe("/helelani/left_motor", 10, &NavigationDriver::_leftMotorCallback, this)),
      m_rightMotorSub(n.subscribe("/helelani/right_motor", 10, &NavigationDriver::_rightMotorCallback, this)),
      m_imuSub(n.subscribe("/helelani/imu", 10, &NavigationDriver::_imuCallback, this))
    {
        m_pwm.begin();
        m_pwm.set_pwm_freq(50);

        setSadl(0);
        setLMotor(0.f);
        setRMotor(0.f);
        setBrake(true);
    }

    float cachedThrottle(float inThrottle)
    {
        inThrottle = std::max(inThrottle, 0.f);
        if (inThrottle > 0.f)
            m_cachedThrottle = inThrottle;
        return m_cachedThrottle;
    }

    void setLMotor(float throttle)
    {
        auto duty = uint16_t(ServoMap(-throttle, -100.f, 100.f, 157.f, 344.f));
        m_pwm.set_pin(L_MOTOR_PIN, duty);
    }

    void setRMotor(float throttle)
    {
        auto duty = uint16_t(ServoMap(throttle, -100.f, 100.f,
                                      PWM_MOTOR_MIN, PWM_MOTOR_MAX));
        m_pwm.set_pin(R_MOTOR_PIN, duty);
    }

    void setSadl(int dir)
    {
        uint16_t duty = PWM_MOTOR_NEUTRAL;
        if (dir < 0)
            duty = PWM_MOTOR_MIN;
        else if (dir > 0)
            duty = PWM_MOTOR_MAX;

        m_pwm.set_pin(SADL_PIN, duty);
    }

    void setBrake(bool on)
    {
        auto duty = uint16_t(on ? PWM_MOTOR_MAX : PWM_MOTOR_MIN);
        m_pwm.set_pin(BRAKE_PIN, duty);
    }

    void clear()
    {
        m_cmds.clear();
    }

    void clearAndStop()
    {
        clear();
        setLMotor(0.f);
        setRMotor(0.f);
        setSadl(0);
        setBrake(true);
    }

    void update(const ros::TimerEvent& e)
    {
        if (m_cmds.empty())
            return;
        double dt = (e.current_expected - e.last_expected).toSec();
        if (m_cmds.front()->update(*this, dt))
            m_cmds.pop_front();
    }

    void enqueueCommand(std::unique_ptr<ICommand>&& cmd)
    {
        m_cmds.push_back(std::move(cmd));
    }

    float leftRotations() const { return m_leftRotations; }
    float rightRotations() const { return m_rightRotations; }
    double imuPitch() const { return m_imuPitch; }
    double imuYaw() const { return m_imuYaw; }
};

class BrakeAfterDelayCommand : public ICommand
{
    double m_remTime;
public:
    BrakeAfterDelayCommand(NavigationDriver& driver, double delay)
    : m_remTime(delay)
    {}

    bool update(NavigationDriver& driver, double dt) override
    {
        m_remTime -= dt;
        if (m_remTime <= 0.0)
        {
            driver.setBrake(true);
            return true;
        }
        return false;
    }
};

class DriveCommand : public ICommand
{
    enum class Unit
    {
        Seconds,
        Rotations,
        Meters
    };
    Unit m_unit;
    float m_remVal;
    float m_throttle;
    float m_initialLeftRotation;
    float m_initialRightRotation;
    bool m_started = false;

    void stop(NavigationDriver& driver)
    {
        driver.setLMotor(0.f);
        driver.setRMotor(0.f);
        driver.enqueueCommand(std::unique_ptr<BrakeAfterDelayCommand>
                              (new BrakeAfterDelayCommand(driver, 1.0)));
    }

public:
    DriveCommand(NavigationDriver& driver,
                 const helelani_common::DriveCommand& cmd)
    : m_remVal(std::max(0.f, cmd.value)),
      m_throttle(driver.cachedThrottle(cmd.throttle) *
                 (cmd.dir == helelani_common::DriveCommand::DIR_BACKWARD ? -1.f : 1.f))
    {
        switch (cmd.unit)
        {
        case helelani_common::DriveCommand::UNIT_SECONDS:
        default:
            m_unit = Unit::Seconds;
            break;
        case helelani_common::DriveCommand::UNIT_ROTATIONS:
            m_unit = Unit::Rotations;
            break;
        case helelani_common::DriveCommand::UNIT_METERS:
            m_unit = Unit::Meters;
            break;
        }
    }

    bool update(NavigationDriver& driver, double dt) override
    {
        driver.setBrake(false);
        driver.setLMotor(m_throttle);
        driver.setRMotor(m_throttle);

        switch (m_unit)
        {
        case Unit::Seconds:
            m_remVal -= dt;
            if (m_remVal <= 0.f)
            {
                stop(driver);
                return true;
            }
            break;
        case Unit::Rotations:
        {
            if (!m_started)
            {
                m_initialLeftRotation = driver.leftRotations();
                m_initialRightRotation = driver.rightRotations();
                m_started = true;
            }
            float totalLeft = std::fabs(driver.leftRotations() - m_initialLeftRotation);
            float totalRight = std::fabs(driver.rightRotations() - m_initialRightRotation);
            if (std::min(totalLeft, totalRight) >= m_remVal)
            {
                stop(driver);
                return true;
            }
            break;
        }
        case Unit::Meters:
        {
            if (!m_started)
            {
                m_initialLeftRotation = driver.leftRotations();
                m_initialRightRotation = driver.rightRotations();
                m_started = true;
            }
            float totalLeft = std::fabs(driver.leftRotations() - m_initialLeftRotation);
            float totalRight = std::fabs(driver.rightRotations() - m_initialRightRotation);
            if (helelani_common::RotationsToMeters(std::min(totalLeft, totalRight)) >= m_remVal)
            {
                stop(driver);
                return true;
            }
            break;
        }
        }

        return false;
    }
};

class TurnCommand : public ICommand
{
    enum class Unit
    {
        Seconds,
        Rotations,
        Degrees
    };
    Unit m_unit;
    float m_remVal;
    float m_throttle;
    double m_initialHeading;
    bool m_started = false;

    void stop(NavigationDriver& driver)
    {
        driver.setLMotor(0.f);
        driver.setRMotor(0.f);
        driver.enqueueCommand(std::unique_ptr<BrakeAfterDelayCommand>
                              (new BrakeAfterDelayCommand(driver, 1.0)));
    }

public:
    TurnCommand(NavigationDriver& driver,
                const helelani_common::DriveCommand& cmd)
    : m_remVal(std::max(0.f, cmd.value)),
      m_throttle(driver.cachedThrottle(cmd.throttle) *
                 (cmd.dir == helelani_common::DriveCommand::DIR_RIGHT ? 1.f : -1.f))
    {
        switch (cmd.unit)
        {
        case helelani_common::DriveCommand::UNIT_SECONDS:
        default:
            m_unit = Unit::Seconds;
            break;
        case helelani_common::DriveCommand::UNIT_ROTATIONS:
            m_unit = Unit::Rotations;
            break;
        case helelani_common::DriveCommand::UNIT_DEGREES:
            m_unit = Unit::Degrees;
            break;
        }
    }

    bool update(NavigationDriver& driver, double dt) override
    {
        driver.setBrake(false);
        driver.setLMotor(m_throttle);
        driver.setRMotor(-m_throttle);

        switch (m_unit)
        {
        case Unit::Seconds:
            m_remVal -= dt;
            if (m_remVal <= 0.f)
            {
                stop(driver);
                return true;
            }
            break;
        case Unit::Rotations:
        {
            if (!m_started)
            {
                m_initialHeading = driver.imuYaw();
                m_started = true;
            }

            double total;
            if (m_throttle < 0.f) // Turn left
                total = m_initialHeading - driver.imuYaw();
            else // Turn right
                total = driver.imuYaw() - m_initialHeading;

            // Radians to rotations
            total /= 2.0 * M_PI;

            if (total >= m_remVal)
            {
                stop(driver);
                return true;
            }

            break;
        }
        case Unit::Degrees:
        {
            if (!m_started)
            {
                m_initialHeading = driver.imuYaw();
                m_started = true;
            }

            double total;
            if (m_throttle < 0.f) // Turn left
                total = m_initialHeading - driver.imuYaw();
            else // Turn right
                total = driver.imuYaw() - m_initialHeading;

            // Radians to degrees
            total *= 180.0 / M_PI;

            if (total >= m_remVal)
            {
                stop(driver);
                return true;
            }

            break;
        }
        }

        return false;
    }
};

class SadlCommand : public ICommand
{
    int m_dir;
    float m_remVal;
    double m_curTime = 0.0;
    double m_lastTime = 0.0;
    double m_lastTimePitch;
    bool m_started = false;
public:
    SadlCommand(NavigationDriver& driver,
                const helelani_common::DriveCommand& cmd)
    : m_remVal(cmd.value)
    {
        switch (cmd.dir)
        {
        case helelani_common::DriveCommand::DIR_UP:
            m_dir = 1;
            break;
        case helelani_common::DriveCommand::DIR_DOWN:
            m_dir = -1;
            break;
        case helelani_common::DriveCommand::DIR_AUTOLEVEL:
        default:
            m_dir = 0;
            break;
        }
    }

    bool update(NavigationDriver& driver, double dt) override
    {
        if (m_dir != 0)
        {
            // Seconds drive
            driver.setSadl(m_dir);
            m_remVal -= dt;
            if (m_remVal <= 0.f)
            {
                driver.setSadl(0);
                return true;
            }
        }
        else
        {
            // Autolevel drive
            double pitchAng = driver.imuPitch() * 180.0 / M_PI;
            if (!m_started)
            {
                m_lastTimePitch = pitchAng;
                m_started = true;
            }
            m_curTime += dt;

            // Stop failsafe for stuck/limited SADL
            if (m_curTime - m_lastTime >= 2.0)
            {
                m_lastTime = m_curTime;
                if (std::fabs(m_lastTimePitch - pitchAng) < 0.5)
                {
                    driver.setSadl(0);
                    return true;
                }
                m_lastTimePitch = pitchAng;
            }

            if (pitchAng < m_remVal)
                driver.setSadl(1);
            else
                driver.setSadl(-1);
            if (std::fabs(pitchAng - m_remVal) < 0.1)
            {
                driver.setSadl(0);
                return true;
            }
        }

        return false;
    }
};

void NavigationDriver::_cmdCallback(const helelani_common::DriveCommand& cmd)
{
    switch (cmd.cmd)
    {
    case helelani_common::DriveCommand::CMD_DRIVE:
        m_cmds.emplace_back(new DriveCommand(*this, cmd));
        break;
    case helelani_common::DriveCommand::CMD_TURN:
        m_cmds.emplace_back(new TurnCommand(*this, cmd));
        break;
    case helelani_common::DriveCommand::CMD_SADL:
        m_cmds.emplace_back(new SadlCommand(*this, cmd));
        break;
    case helelani_common::DriveCommand::CMD_KILL:
        clearAndStop();
        break;
    default:
        break;
    }
}

void NavigationDriver::_throttleCallback(const helelani_common::Throttle& cmd)
{
    clear();

    setBrake(false);
    setLMotor(cmd.left * 100.f);
    setRMotor(cmd.right * 100.f);

    if (cmd.left == 0.f && cmd.right == 0.f)
        m_cmds.emplace_back(new BrakeAfterDelayCommand(*this, 1.0));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle n;

    std::string lower_i2c_path, lower_iio_path;
    int gpio_base;
    find_lower_dln(lower_i2c_path, lower_iio_path, gpio_base);
    I2CInterface pwmI2C(lower_i2c_path, PWM_ADDR);
    if (!pwmI2C)
        return 1;

    NavigationDriver driver(pwmI2C, n);

    // Begin update loop
    ros::Timer t = n.createTimer(ros::Duration(0.02),
                                 &NavigationDriver::update, &driver);
    ros::spin();

    driver.clearAndStop();

    return 0;
}
