#ifndef L3G4200DRIVER_H
#define L3G4200DRIVER_H

#include "i2c.h"

class L3G
{
public:
    template <typename T> struct vector
    {
        T x, y, z;
    };

    enum deviceType { device_4200D, device_D20, device_D20H, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    // register addresses
    enum regAddr
    {
        WHO_AM_I       = 0x0F,

        CTRL1          = 0x20, // D20H
        CTRL_REG1      = 0x20, // D20, 4200D
        CTRL2          = 0x21, // D20H
        CTRL_REG2      = 0x21, // D20, 4200D
        CTRL3          = 0x22, // D20H
        CTRL_REG3      = 0x22, // D20, 4200D
        CTRL4          = 0x23, // D20H
        CTRL_REG4      = 0x23, // D20, 4200D
        CTRL5          = 0x24, // D20H
        CTRL_REG5      = 0x24, // D20, 4200D
        REFERENCE      = 0x25,
        OUT_TEMP       = 0x26,
        STATUS         = 0x27, // D20H
        STATUS_REG     = 0x27, // D20, 4200D

        OUT_X_L        = 0x28,
        OUT_X_H        = 0x29,
        OUT_Y_L        = 0x2A,
        OUT_Y_H        = 0x2B,
        OUT_Z_L        = 0x2C,
        OUT_Z_H        = 0x2D,

        FIFO_CTRL      = 0x2E, // D20H
        FIFO_CTRL_REG  = 0x2E, // D20, 4200D
        FIFO_SRC       = 0x2F, // D20H
        FIFO_SRC_REG   = 0x2F, // D20, 4200D

        IG_CFG         = 0x30, // D20H
        INT1_CFG       = 0x30, // D20, 4200D
        IG_SRC         = 0x31, // D20H
        INT1_SRC       = 0x31, // D20, 4200D
        IG_THS_XH      = 0x32, // D20H
        INT1_THS_XH    = 0x32, // D20, 4200D
        IG_THS_XL      = 0x33, // D20H
        INT1_THS_XL    = 0x33, // D20, 4200D
        IG_THS_YH      = 0x34, // D20H
        INT1_THS_YH    = 0x34, // D20, 4200D
        IG_THS_YL      = 0x35, // D20H
        INT1_THS_YL    = 0x35, // D20, 4200D
        IG_THS_ZH      = 0x36, // D20H
        INT1_THS_ZH    = 0x36, // D20, 4200D
        IG_THS_ZL      = 0x37, // D20H
        INT1_THS_ZL    = 0x37, // D20, 4200D
        IG_DURATION    = 0x38, // D20H
        INT1_DURATION  = 0x38, // D20, 4200D

        LOW_ODR        = 0x39  // D20H
    };

    vector<int16_t> g; // gyro angular velocity readings

    L3G(I2CInterface& interface);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);

    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    bool read();

    void setTimeout(unsigned int timeout);
    unsigned int getTimeout(void);
    bool timeoutOccurred(void);

private:
    I2CInterface& _handle;
    deviceType _device; // chip type (D20H, D20, or 4200D)
    uint8_t address;

    unsigned int io_timeout;
    bool did_timeout;

    int testReg(uint8_t address, regAddr reg);
};

#endif // L3G4200DRIVER_H
