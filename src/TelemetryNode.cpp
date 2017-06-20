#include <ros/ros.h>
#include <helelani_common/Imu.h>
#include <helelani_common/Motors.h>

#include <sys/types.h>
#include <fcntl.h>

#include "DlnFinders.h"
#include "i2c.h"
#include "analog.h"
#include "GPSReader.h"

#include "Hmc5883lDriver.h"
#include "AdxlDriver.h"
#include "Bmp085Driver.h"
#include "CANTelemetry.h"
#include "GPIODriver.h"

class RoverTelemetry
{
    IIOAnalogInterface& m_lowerAnalog;
    AdxlDriver m_adxl;
    Hmc5883lDriver m_mag;
    Bmp085Driver m_bmp;
    GPSReader m_gps;
    CANMotorData m_leftMotor;
    CANMotorData m_rightMotor;
    GPIODriver m_gpio;

    helelani_common::Imu m_imu;
    ros::Publisher m_imuPub;
    ros::Publisher m_motorPub;
public:
    RoverTelemetry(ros::NodeHandle& n,
                   I2CInterface& upperAxdlIntf,
                   I2CInterface& upperMagIntf,
                   I2CInterface& upperBmp,
                   IIOAnalogInterface& lowerAnalog,
                   const std::string& gpsPath,
                   int gpioBase)
    : m_lowerAnalog(lowerAnalog),
      m_adxl(upperAxdlIntf),
      m_mag(upperMagIntf),
      m_bmp(upperBmp),
      m_gps(n, gpsPath),
      m_leftMotor("canLeft"),
      m_rightMotor("canRight"),
      m_gpio(gpioBase),
      m_imuPub(n.advertise<helelani_common::Imu>("/helelani/imu", 1000)),
      m_motorPub(n.advertise<helelani_common::Motors>("/helelani/motors", 1000))
    {
        m_adxl.initialize();
        m_adxl.setOffsetZ(7);
        m_mag.initialize();
        m_bmp.initialize();
        m_bmp.setControl(BMP085_MODE_TEMPERATURE);
    }

    void update()
    {
        m_imu.accel.x = m_adxl.getAccelerationX();
        m_imu.accel.y = m_adxl.getAccelerationY();
        m_imu.accel.z = m_adxl.getAccelerationZ();

        m_imu.mag.x = m_mag.getHeadingX();
        m_imu.mag.y = m_mag.getHeadingY();
        m_imu.mag.z = m_mag.getHeadingZ();

        if (m_bmp.getControl() == 0xd0) {
            m_imu.pressure = m_bmp.getPressure();
            m_imu.altitude = m_bmp.getAltitude(m_imu.pressure);
            m_bmp.setControl(BMP085_MODE_TEMPERATURE);
        } else if (m_bmp.getControl() == 0xa) {
            m_imu.temperature = m_bmp.getTemperatureF();
            m_bmp.setControl(BMP085_MODE_PRESSURE_3);
        }

        m_imu.header.stamp = ros::Time::now();
        m_imuPub.publish(m_imu);

        m_gps.PublishReading();

        helelani_common::Motors motorsOut;
        SCANMotorData leftMotor = m_leftMotor.getData();
        SCANMotorData rightMotor = m_rightMotor.getData();

        bool leftHiGear = m_gpio.readPin(24);
        bool rightHiGear = m_gpio.readPin(25);

        motorsOut.left_current = leftMotor.getCurrent();
        motorsOut.left_speed = leftMotor.getSpeed(leftHiGear);
        motorsOut.left_higear = leftHiGear;
        motorsOut.right_current = rightMotor.getCurrent();
        motorsOut.right_speed = rightMotor.getSpeed(rightHiGear);
        motorsOut.right_higear = rightHiGear;

        motorsOut.header.stamp = ros::Time::now();
        m_motorPub.publish(motorsOut);
    }
};

int main(int argc, char *argv[])
{
    // Setup ROS
    ros::init(argc, argv, "telemetry_node");
    ros::NodeHandle n;

    // Find upper DLN
    std::string upper_i2c_path;
    //if (!find_upper_dln(upper_i2c_path))
    //    return -1;

    // Find lower DLN
    std::string lower_i2c_path, iio_root;
    int gpio_base = -1;
    if (!find_lower_dln(lower_i2c_path, iio_root, gpio_base))
        return -1;

    // Find GPS
    std::string gps_path;
    if (!find_gps(gps_path))
        return -1;

    ROS_INFO("\nUpper I2C: %s\nLower I2C: %s\nLower ADC: %s\nGPS: %s",
             upper_i2c_path.c_str(), lower_i2c_path.c_str(),
             iio_root.c_str(), gps_path.c_str());

    // Open upper I2C
    I2CInterface upper_axdl(upper_i2c_path, ADXL345_DEFAULT_ADDRESS);
    if (!upper_axdl)
        return -1;
    I2CInterface upper_mag(upper_i2c_path, HMC5883L_ADDRESS);
    if (!upper_mag)
        return -1;
    I2CInterface upper_bmp(upper_i2c_path, BMP085_ADDRESS);
    if (!upper_bmp)
        return -1;

    // Lower analog interface
    IIOAnalogInterface lower_analog(iio_root);

    // Construct telemetry class
    RoverTelemetry tele(n, upper_axdl, upper_mag, upper_bmp,
                        lower_analog, gps_path, gpio_base);

    // Begin update loop
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        tele.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
