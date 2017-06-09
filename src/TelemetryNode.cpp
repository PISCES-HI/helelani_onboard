#include <ros/ros.h>
#include <helelani_common/Imu.h>

#include "dln/dln.h"
#include "dln/dln_generic.h"
#include "dln/dln_gpio.h"

#include "i2c.h"
#include "analog.h"

#include "Hmc5883lDriver.h"
#include "AdxlDriver.h"

class RoverTelemetry
{
    HDLN& m_dln;
    AdxlDriver m_adxl;
    Hmc5883lDriver m_mag;

    ros::Publisher m_imuPub;
public:
    RoverTelemetry(ros::NodeHandle& n, HDLN& dln)
    : m_dln(dln),
      m_adxl(m_dln),
      m_mag(m_dln),
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

    // Setup DLN
    DLN_RESULT result;
    DlnConnect("localhost", DLN_DEFAULT_SERVER_PORT);

    // Check device count
    uint32_t deviceCount;
    result = DlnGetDeviceCount(&deviceCount);
    if (!DLN_SUCCEEDED(result)) {
        ROS_ERROR("Failed to get DLN device count");
        return -1;
    }
    if (deviceCount != 1) {
        ROS_ERROR("There should be one and only one DLN device connected. There are %d", int(deviceCount));
        return -1;
    }

    // Try to open our device
    HDLN handle;
    result = DlnOpenDevice(0, &handle);
    if (!DLN_SUCCEEDED(result)) {
        ROS_ERROR("Failed to open DLN device");
        return -1;
    }

    // Print out some info about our DLN device
    DLN_VERSION version;
    uint32_t sn, id;
    DlnGetVersion(handle, &version);
    DlnGetDeviceSn(handle, &sn);
    DlnGetDeviceId(handle, &id);
    ROS_INFO("Opened DLN device: %d %d", sn, id);

    // Initialize all the things
    init_i2c(handle);
    init_analog(handle);

    // Construct telemetry class
    RoverTelemetry tele(n);

    // Begin update loop
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        tele.update();
        ros::spinOnce();
        r.sleep();
    }

    // Shutdown
    cleanup_analog(handle);
    DlnCloseHandle(handle);
    DlnDisconnectAll();

    return 0;
}
