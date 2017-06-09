#include <ros/console.h>

#include "dln/dln_generic.h"
#include "dln/dln_adc.h"

#include "analog.h"

void init_analog(HDLN handle) {
    DLN_RESULT result;

    // Get port count
    uint8_t port_count;
    result = DlnAdcGetPortCount(handle, &port_count);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to get ADC port count: %d", result);
    }
    ROS_INFO("ADC ports: %d", int(port_count));

    // Get channel count
    uint8_t channel_count;
    result = DlnAdcGetChannelCount(handle, 0, &channel_count);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to get ADC channel count: %d", result);
    }
    ROS_INFO("ADC channels: %d", int(channel_count));

    DlnAdcDisable(handle, 0);

    result = DlnAdcSetResolution(handle, 0, DLN_ADC_RESOLUTION_10BIT);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to set ADC port 0 resolution: %d", result);
    }

    // Enable port 0 with channel 0
    result = DlnAdcChannelEnable(handle, 0, 0);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 0: %d", result);
    }
    result = DlnAdcChannelEnable(handle, 0, 1);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 1: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 2);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 2: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 3);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 3: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 4);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 4: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 5);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 5: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 6);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 6: %d", result);
    }
    DlnAdcChannelEnable(handle, 0, 7);
    if (DLN_FAILED(result)) {
        ROS_ERROR("Failed to enable channel port 7: %d", result);
    }

    // Enable port 0
    uint16_t conflict;
    result = DlnAdcEnable(handle, 0, &conflict);
    if (DLN_FAILED(result)) {
        ROS_ERROR("DlnAdcEnable() port 0 error ", result);
        return;
    }
}

void cleanup_analog(HDLN handle) {
    DlnAdcDisable(handle, 0);
    DlnAdcDisable(handle, 1);
    DlnAdcDisable(handle, 2);
    DlnAdcDisable(handle, 3);
    DlnAdcDisable(handle, 4);
    DlnAdcDisable(handle, 5);
    DlnAdcDisable(handle, 6);
    DlnAdcDisable(handle, 7);
    DlnAdcChannelDisable(handle, 0, 0);
    DlnAdcChannelDisable(handle, 0, 1);
    DlnAdcChannelDisable(handle, 0, 2);
    DlnAdcChannelDisable(handle, 0, 3);
    DlnAdcChannelDisable(handle, 0, 4);
    DlnAdcChannelDisable(handle, 0, 5);
    DlnAdcChannelDisable(handle, 0, 6);
    DlnAdcChannelDisable(handle, 0, 7);
}

float read_voltage(const uint16_t analog_reading, const float r1, const float r2) {
    float scaled_reading = (((float)analog_reading) / 1024.f) * 5.f; // Normalize and scale by Vcc (5v)

    return (scaled_reading / r2) * (r1 + r2);
}

float get_48v_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 17868.0, 950.0);
}

float get_12v_e_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 19890.0, 1899.0);
}

float get_12v_pl_voltage(const uint16_t analog_reading) {
    return read_voltage(analog_reading, 19890.0, 1899.0);
}

float get_avionics_temperature(const uint16_t analog_reading) {
    // Get voltage (between 0 and 4.5v)
    float voltage = (float(analog_reading)/1023.0) * 4.5;
    // 9.8mv per degree and offset by -30 C
    return ((voltage * 1000.0) / 9.8) - 30.0;
}


float get_ambient_temperature(const uint16_t analog_reading) {
    // Get voltage (between 0 and 4.5v)
    float voltage = (float(analog_reading)/1023.0) * 4.5;
    // 9.8mv per degree and offset by -15 C
    return ((voltage * 1000.0) / 9.8) - 15.0;
}
