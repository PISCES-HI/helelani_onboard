#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <helelani_common/Imu.h>
#include <helelani_common/Motors.h>
#include <helelani_common/Analog.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include <signal.h>

#include "DlnFinders.h"
#include "i2c.h"
#include "Analog.h"
#include "GPSReader.h"

#include "Hmc5883lDriver.h"
#include "AdxlDriver.h"
#include "L3G4200Driver.h"
#include "Bmp085Driver.h"
#include "CANTelemetry.h"
#include "GPIODriver.h"

#define GYRO_TARE_X (-102.0)
#define GYRO_TARE_Y (-458.0)
#define GYRO_TARE_Z (-7.0)
#define ACCEL_ALPHA 0.5
#define GYRO_ALPHA 0.5
#define ANALOG_ALPHA 0.05f

static void SigUsrHandler(int) {}

class RoverTelemetry
{
    std::mutex m_gyroLock;
    std::thread m_gyroThread;

    AdxlDriver m_adxl;
    L3G m_gyro;
    Hmc5883lDriver m_mag;
    Bmp085Driver m_bmp;
    CANMotorData m_leftMotor;
    CANMotorData m_rightMotor;
    GPIODriver m_gpio;

    helelani_common::Imu m_imu = {};
    ros::Publisher m_rosimuPub;
    ros::Publisher m_imuPub;
    ros::Publisher m_motorPub;
    ros::Publisher m_analogPub;
    ros::ServiceServer m_recalibrateSrv;
    IIOAnalogInterface m_lowerAnalog;

    bool m_running = true;

public:
    RoverTelemetry(ros::NodeHandle& n,
                   I2CInterface& upperAxdlIntf,
                   I2CInterface& upperGyroIntf,
                   I2CInterface& upperMagIntf,
                   I2CInterface& upperBmp,
                   const std::string& lowerAnalog,
                   int gpioBase)
    : m_adxl(upperAxdlIntf),
      m_gyro(upperGyroIntf),
      m_mag(upperMagIntf),
      m_bmp(upperBmp),
      m_leftMotor("canLeft"),
      m_rightMotor("canRight"),
      m_gpio(gpioBase),
      m_rosimuPub(n.advertise<sensor_msgs::Imu>("/helelani/rosimu", 1000)),
      m_imuPub(n.advertise<helelani_common::Imu>("/helelani/imu", 1000)),
      m_motorPub(n.advertise<helelani_common::Motors>("/helelani/motors", 1000)),
      m_analogPub(n.advertise<helelani_common::Analog>("/helelani/analog", 1000)),
      m_recalibrateSrv(n.advertiseService("/helelani/recalibrate",
                                          &RoverTelemetry::recalibrate, this)),
      m_lowerAnalog(lowerAnalog, std::bind(&RoverTelemetry::updateAnalog,
                                           this, std::placeholders::_1))
    {
        m_adxl.initialize();
        m_adxl.setOffsetZ(7);
        m_gyro.init(L3G::deviceType::device_4200D, L3G::sa0_high);
        m_gyro.enableDefault();
        m_mag.initialize();
        m_bmp.initialize();
        m_bmp.setControl(BMP085_MODE_TEMPERATURE);

        m_gyroThread = std::thread([this]()
        {
            ros::Rate r(300);

            double fXg = 0.0;
            double fYg = 0.0;
            double fZg = 0.0;

            double fXa = 0.0;
            double fYa = 0.0;
            double fZa = 0.0;

            double accelPitch = 0.0;
            double accelRoll = 0.0;
            double oldYaw = 0.0;
            double yawPeriod = 0.0;
            double magYaw = 0.0;

            double finalRoll = 0.0;
            double finalPitch = 0.0;
            double finalYaw = 0.0;

            sensor_msgs::Imu rosImu = {};

            while (m_running)
            {
                // Accelerometer (pitch/roll)
                if (m_adxl.getIntDataReadySource())
                {
                    int16_t accelX, accelY, accelZ;
                    m_adxl.getAcceleration(&accelX, &accelY, &accelZ);

                    // Low Pass Filter
                    fXa = accelX * ACCEL_ALPHA + (fXa * (1.0 - ACCEL_ALPHA));
                    fYa = accelY * ACCEL_ALPHA + (fYa * (1.0 - ACCEL_ALPHA));
                    fZa = accelZ * ACCEL_ALPHA + (fZa * (1.0 - ACCEL_ALPHA));

                    rosImu.linear_acceleration.x = fXa / 255.0 * 9.8;
                    rosImu.linear_acceleration.y = fYa / 255.0 * 9.8;
                    rosImu.linear_acceleration.z = -fZa / 255.0 * 9.8;

                    // Roll & Pitch Equations
                    accelRoll = std::atan2(-fYa, fZa);
                    accelPitch = std::atan2(fXa, std::sqrt(fYa * fYa + fZa * fZa));
                }

                // Magnetometer (yaw)
                if (m_mag.getReadyStatus())
                {
                    int16_t headingX, headingY, headingZ;
                    m_mag.getHeading(&headingX, &headingY, &headingZ);
                    double newYaw = std::atan2(headingY, headingX) + 20.f * M_PI / 180.f;
                    if (newYaw >= 0.0 && oldYaw < 0.0)
                    {
                        // Correct neg-to-pos discontinuity
                        if (newYaw - (M_PI / 2.0) > 0.0 && -oldYaw - (M_PI / 2.0) > 0.0)
                            yawPeriod -= 2.0 * M_PI;
                    }
                    else if (newYaw < 0.0 && oldYaw >= 0.0)
                    {
                        // Correct pos-to-neg discontinuity
                        if (-newYaw - (M_PI / 2.0) > 0.0 && oldYaw - (M_PI / 2.0) > 0.0)
                            yawPeriod += 2.0 * M_PI;
                    }
                    oldYaw = newYaw;
                    magYaw = newYaw + yawPeriod;
                }

                // Gyro (pitch/roll/yaw motion)
                if (m_gyro.read())
                {
                    double x = (m_gyro.g.x + GYRO_TARE_X) * 0.00875 * (M_PI / 180.0);
                    double y = (m_gyro.g.y + GYRO_TARE_Y) * 0.00875 * (M_PI / 180.0);
                    double z = (m_gyro.g.z + GYRO_TARE_Z) * 0.00875 * (M_PI / 180.0);

                    // Low Pass Filter
                    fXg = x * GYRO_ALPHA + (fXg * (1.0 - GYRO_ALPHA));
                    fYg = y * GYRO_ALPHA + (fYg * (1.0 - GYRO_ALPHA));
                    fZg = z * GYRO_ALPHA + (fZg * (1.0 - GYRO_ALPHA));

                    rosImu.angular_velocity.x = fXg;
                    rosImu.angular_velocity.y = fYg;
                    rosImu.angular_velocity.z = fZg;

                    // Integrate with complimentary filter
                    finalRoll = 0.95 * (finalRoll + fXg / 200.0) + 0.05 * accelRoll;
                    finalPitch = 0.95 * (finalPitch + fYg / 200.0) + 0.05 * accelPitch;
                    finalYaw = 0.95 * (finalYaw - fZg / 200.0) + 0.05 * magYaw;

                    tf2::Quaternion quat;
                    quat.setEuler(finalYaw, finalPitch, finalRoll);
                    rosImu.orientation.w = quat.getW();
                    rosImu.orientation.x = quat.getX();
                    rosImu.orientation.y = quat.getY();
                    rosImu.orientation.z = quat.getZ();

                    rosImu.header.stamp = ros::Time::now();
                    m_rosimuPub.publish(rosImu);

                    std::lock_guard<std::mutex> lk(m_gyroLock);
                    m_imu.roll = finalRoll;
                    m_imu.pitch = finalPitch;
                    m_imu.yaw = finalYaw;
                }
                r.sleep();
            }
        });
    }

    ~RoverTelemetry()
    {
        m_running = false;
        if (m_gyroThread.joinable())
            m_gyroThread.join();
    }

    // Low Pass Filter
    float m_analogFiltered[8] = {};
    float filterAnalogChannel(const uint16_t* readings, int idx)
    {
        m_analogFiltered[idx] = readings[idx] * ANALOG_ALPHA +
                (m_analogFiltered[idx] * (1.f - ANALOG_ALPHA));
        return m_analogFiltered[idx];
    }

    void updateAnalog(const uint16_t* readings)
    {
        helelani_common::Analog msg = {};

        msg.current_12 = (filterAnalogChannel(readings, 0) - 84.f) * 840.f / 35000.f;
        msg.voltage_12 = filterAnalogChannel(readings, 1) * 0.043448f;
        msg.voltage_48 = filterAnalogChannel(readings, 3) * 0.0678303f;
        msg.current_24 = (filterAnalogChannel(readings, 4) - 84.f) * 840.f / 35000.f;
        msg.temp_l = filterAnalogChannel(readings, 6);
        msg.temp_r = filterAnalogChannel(readings, 2);

        msg.header.stamp = ros::Time::now();
        m_analogPub.publish(msg);
    }

    bool recalibrate(std_srvs::Empty::Request& request,
                     std_srvs::Empty::Response& response)
    {
        return true;
    }

    void update()
    {
        if (m_bmp.getControl() == 0xd0) {
            m_imu.pressure = m_bmp.getPressure();
            m_imu.altitude = m_bmp.getAltitude(m_imu.pressure);
            m_bmp.setControl(BMP085_MODE_TEMPERATURE);
        } else if (m_bmp.getControl() == 0xa) {
            m_imu.temperature = m_bmp.getTemperatureF();
            m_bmp.setControl(BMP085_MODE_PRESSURE_3);
        }

        std::lock_guard<std::mutex> lk(m_gyroLock);
        m_imu.header.stamp = ros::Time::now();
        m_imuPub.publish(m_imu);

        helelani_common::Motors motorsOut;
        SCANMotorData leftMotor = m_leftMotor.getData();
        SCANMotorData rightMotor = m_rightMotor.getData();

        bool leftHiGear = m_gpio.readPin(0);
        bool rightHiGear = m_gpio.readPin(1);

        motorsOut.left_current = leftMotor.getCurrent();
        motorsOut.left_speed = leftMotor.getSpeed(leftHiGear);
        motorsOut.left_higear = uint8_t(leftHiGear);
        motorsOut.right_current = rightMotor.getCurrent();
        motorsOut.right_speed = rightMotor.getSpeed(rightHiGear);
        motorsOut.right_higear = uint8_t(rightHiGear);

        motorsOut.header.stamp = ros::Time::now();
        m_motorPub.publish(motorsOut);
    }
};

int main(int argc, char *argv[])
{
    // Register dummy SIGUSR1 for terminating threads
    struct sigaction sa = {};
    sa.sa_handler = SigUsrHandler;
    sigaction(SIGUSR1, &sa, nullptr);

    // Setup ROS
    ros::init(argc, argv, "telemetry_node");
    ros::NodeHandle n;

    // Find upper DLN
    std::string upper_i2c_path;
    find_upper_dln(upper_i2c_path);

    // Find lower DLN
    std::string lower_i2c_path, lower_analog;
    int gpio_base = -1;
    find_lower_dln(lower_i2c_path, lower_analog, gpio_base);

    // Find GPS
    std::string gps_path;
    find_gps(gps_path);

    ROS_INFO("\nUpper I2C: %s\nLower I2C: %s\nLower ADC: %s\nGPS: %s",
             upper_i2c_path.c_str(), lower_i2c_path.c_str(),
             lower_analog.c_str(), gps_path.c_str());

    // Open upper I2C
    I2CInterface upper_axdl(upper_i2c_path, ADXL345_DEFAULT_ADDRESS);
    I2CInterface upper_gyro(upper_i2c_path);
    I2CInterface upper_mag(upper_i2c_path, HMC5883L_ADDRESS);
    I2CInterface upper_bmp(upper_i2c_path, BMP085_ADDRESS);

    // Construct telemetry class
    RoverTelemetry tele(n, upper_axdl, upper_gyro, upper_mag, upper_bmp,
                        lower_analog, gpio_base);

    // Start GPS reader
    GPSReader gps(n, gps_path);

    // Begin update loop
    ros::Rate r(50); // 50 hz
    while (ros::ok())
    {
        tele.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
