#include "DlnFinders.h"
#include <ros/console.h>

#define UPPER_DLN_PATH "/sys/devices/pci0000:00/0000:00:14.0/usb1/1-4/1-4:1.0"
#define LOWER_DLN_PATH "/sys/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.0"

bool find_upper_dln(std::string& i2c_path)
{
    struct udev* udev = udev_new();
    if (!udev) {
        ROS_ERROR("Can't create udev");
        return false;
    }

    struct udev_device* lower_avionics_dev =
        udev_device_new_from_syspath(udev, UPPER_DLN_PATH);
    if (!lower_avionics_dev) {
        ROS_ERROR("Can't find upper avionics DLN at %s", UPPER_DLN_PATH);
        udev_unref(udev);
        return false;
    }

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_parent(enumerate, lower_avionics_dev);
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* dev_list_entry;
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char* path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device* chdev = udev_device_new_from_syspath(udev, path);
        const char* subsystem = udev_device_get_subsystem(chdev);
        if (subsystem) {
            if (!strcmp(subsystem, "i2c-dev")) {
                i2c_path = udev_device_get_devnode(chdev);
                udev_device_unref(chdev);
                break;
            }
        }
        udev_device_unref(chdev);
    }
    udev_enumerate_unref(enumerate);

    udev_unref(udev);
    return true;
}

bool find_lower_dln(std::string& i2c_path, std::string& iio_root)
{
    struct udev* udev = udev_new();
    if (!udev) {
        ROS_ERROR("Can't create udev");
        return false;
    }

    struct udev_device* lower_avionics_dev =
        udev_device_new_from_syspath(udev, LOWER_DLN_PATH);
    if (!lower_avionics_dev) {
        ROS_ERROR("Can't find lower avionics DLN at %s", LOWER_DLN_PATH);
        udev_unref(udev);
        return false;
    }

    struct udev_enumerate* enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_parent(enumerate, lower_avionics_dev);
    udev_enumerate_scan_devices(enumerate);
    struct udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    struct udev_list_entry* dev_list_entry;
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char* path = udev_list_entry_get_name(dev_list_entry);
        struct udev_device* chdev = udev_device_new_from_syspath(udev, path);
        const char* subsystem = udev_device_get_subsystem(chdev);
        if (subsystem) {
            if (!strcmp(subsystem, "i2c-dev"))
                i2c_path = udev_device_get_devnode(chdev);
            else if (!strcmp(subsystem, "iio"))
                iio_root = path;
        }
        udev_device_unref(chdev);
    }
    udev_enumerate_unref(enumerate);

    udev_unref(udev);
    return true;
}
