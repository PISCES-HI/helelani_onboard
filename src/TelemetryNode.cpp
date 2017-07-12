#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <helelani_common/Imu.h>
#include <helelani_common/Motors.h>
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

#define GYRO_TARE_X -102.0
#define GYRO_TARE_Y -458.0
#define GYRO_TARE_Z -7.0
#define ACCEL_ALPHA 0.5
#define GYRO_ALPHA 0.5

static void SigUsrHandler(int) {}

class RoverTelemetry
{
    IIOAnalogInterface& m_lowerAnalog;
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
    ros::ServiceServer m_recalibrateSrv;

    std::mutex m_gyroLock;
    std::thread m_gyroThread;
    bool m_running = true;

public:
    RoverTelemetry(ros::NodeHandle& n,
                   I2CInterface& upperAxdlIntf,
                   I2CInterface& upperGyroIntf,
                   I2CInterface& upperMagIntf,
                   I2CInterface& upperBmp,
                   IIOAnalogInterface& lowerAnalog,
                   int gpioBase)
    : m_lowerAnalog(lowerAnalog),
      m_adxl(upperAxdlIntf),
      m_gyro(upperGyroIntf),
      m_mag(upperMagIntf),
      m_bmp(upperBmp),
      m_leftMotor("canLeft"),
      m_rightMotor("canRight"),
      m_gpio(gpioBase),
      m_rosimuPub(n.advertise<sensor_msgs::Imu>("/helelani/rosimu", 1000)),
      m_imuPub(n.advertise<helelani_common::Imu>("/helelani/imu", 1000)),
      m_motorPub(n.advertise<helelani_common::Motors>("/helelani/motors", 1000)),
      m_recalibrateSrv(n.advertiseService("/helelani/recalibrate",
                                          &RoverTelemetry::recalibrate, this))
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
            double magYaw = 0.0;

            double finalRoll = 0.0;
            double finalPitch = 0.0;
            double finalYaw = 0.0;

            sensor_msgs::Imu rosImu = {};

            while (m_running)
            {
                // Accelerometer (pitch/roll)
                bool ready = m_adxl.getIntDataReadySource();
                if (ready)
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
                ready = m_mag.getReadyStatus();
                if (ready)
                {
                    int16_t headingX, headingY, headingZ;
                    m_mag.getHeading(&headingX, &headingY, &headingZ);
                    magYaw = std::atan2(headingY, headingX);
                }

                // Gyro (pitch/roll/yaw motion)
                m_gyro.read(ready);
                if (ready)
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

    bool recalibrate(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
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

        bool leftHiGear = m_gpio.readPin(24);
        bool rightHiGear = m_gpio.readPin(25);

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
    std::string lower_i2c_path, iio_dev;
    int gpio_base = -1;
    find_lower_dln(lower_i2c_path, iio_dev, gpio_base);

    // Find GPS
    std::string gps_path;
    find_gps(gps_path);

    ROS_INFO("\nUpper I2C: %s\nLower I2C: %s\nLower ADC: %s\nGPS: %s",
             upper_i2c_path.c_str(), lower_i2c_path.c_str(),
             iio_dev.c_str(), gps_path.c_str());

    // Open upper I2C
    I2CInterface upper_axdl(upper_i2c_path, ADXL345_DEFAULT_ADDRESS);
    I2CInterface upper_gyro(upper_i2c_path);
    I2CInterface upper_mag(upper_i2c_path, HMC5883L_ADDRESS);
    I2CInterface upper_bmp(upper_i2c_path, BMP085_ADDRESS);

    // Lower analog interface
    IIOAnalogInterface lower_analog(iio_dev);

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
