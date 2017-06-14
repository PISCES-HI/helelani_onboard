#include <ros/ros.h>
#include <helelani_common/Imu.h>

#include <sys/types.h>
#include <fcntl.h>

#include "DlnFinders.h"
#include "i2c.h"
#include "analog.h"

#include "Hmc5883lDriver.h"
#include "AdxlDriver.h"

class RoverTelemetry
{
    IIOAnalogInterface& m_lowerAnalog;
    AdxlDriver m_adxl;
    Hmc5883lDriver m_mag;

    ros::Publisher m_imuPub;
public:
    RoverTelemetry(ros::NodeHandle& n,
                   I2CInterface& upperAxdlIntf,
                   I2CInterface& upperMagIntf,
                   IIOAnalogInterface& lowerAnalog)
    : m_lowerAnalog(lowerAnalog),
      m_adxl(upperAxdlIntf),
      m_mag(upperMagIntf),
      m_imuPub(n.advertise<helelani_common::Imu>("/helelani/imu", 1000))
    {
        m_adxl.initialize();
        m_adxl.setOffsetZ(7);
        m_mag.initialize();
    }

    void update()
    {
        helelani_common::Imu imu;
        imu.accel.x = m_adxl.getAccelerationX();
        imu.accel.y = m_adxl.getAccelerationY();
        imu.accel.z = m_adxl.getAccelerationZ();
        imu.mag.x = m_mag.getHeadingX();
        imu.mag.y = m_mag.getHeadingY();
        imu.mag.z = m_mag.getHeadingZ();
        m_imuPub.publish(imu);
    }
};

int main(int argc, char *argv[])
{
    // Setup ROS
    ros::init(argc, argv, "telemetry_node");
    ros::NodeHandle n;

    // Find upper DLN
    std::string upper_i2c_path;
    if (!find_upper_dln(upper_i2c_path))
        return -1;

    // Find lower DLN
    std::string lower_i2c_path, iio_root;
    if (!find_lower_dln(lower_i2c_path, iio_root))
        return -1;

    ROS_INFO("\nUpper I2C: %s\nLower I2C: %s\nLower ADC: %s", upper_i2c_path.c_str(),
             lower_i2c_path.c_str(), iio_root.c_str());

    // Open upper I2C
    int upper_i2c_fd = open(upper_i2c_path.c_str(), O_RDWR);
    if (!upper_i2c_fd) {
        ROS_ERROR("Unable to open %s: %s", upper_i2c_path.c_str(), strerror(errno));
        return -1;
    }
    I2CInterface upper_axdl(upper_i2c_fd, ADXL345_DEFAULT_ADDRESS);
    I2CInterface upper_mag(upper_i2c_fd, HMC5883L_ADDRESS);

    // Lower analog interface
    IIOAnalogInterface lower_analog(iio_root);

    // Construct telemetry class
    RoverTelemetry tele(n, upper_axdl, upper_mag, lower_analog);

    // Begin update loop
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        tele.update();
        ros::spinOnce();
        r.sleep();
    }

    // Shutdown
    close(upper_i2c_fd);

    return 0;
}
