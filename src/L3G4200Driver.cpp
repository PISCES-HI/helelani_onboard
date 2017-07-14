#include <ros/console.h>
#include <tf/LinearMath/Vector3.h>
#include <cmath>
#include <unistd.h>

#include "i2c.h"
#include "L3G4200Driver.h"

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define D20_SA0_HIGH_ADDRESS      0b1101011 // also applies to D20H
#define D20_SA0_LOW_ADDRESS       0b1101010 // also applies to D20H
#define L3G4200D_SA0_HIGH_ADDRESS 0b1101001
#define L3G4200D_SA0_LOW_ADDRESS  0b1101000

#define TEST_REG_ERROR -1

#define D20H_WHO_ID     0xD7
#define D20_WHO_ID      0xD4
#define L3G4200D_WHO_ID 0xD3

// Constructors ////////////////////////////////////////////////////////////////

L3G::L3G(I2CInterface& interface)
: _handle(interface)
{
    _device = device_auto;

    io_timeout = 0;  // 0 = no timeout
    did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in read() since the last call to timeoutOccurred()?
bool L3G::timeoutOccurred()
{
    bool tmp = did_timeout;
    did_timeout = false;
    return tmp;
}

void L3G::setTimeout(unsigned int timeout)
{
    io_timeout = timeout;
}

unsigned int L3G::getTimeout()
{
    return io_timeout;
}

bool L3G::init(deviceType device, sa0State sa0)
{
    int id;

    // perform auto-detection unless device type and SA0 state were both specified
    if (device == device_auto || sa0 == sa0_auto)
    {
        // check for L3GD20H, D20 if device is unidentified or was specified to be one of these types
        if (device == device_auto || device == device_D20H || device == device_D20)
        {
            // check SA0 high address unless SA0 was specified to be low
            if (sa0 != sa0_low && (id = testReg(D20_SA0_HIGH_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
            {
                // device responds to address 1101011; it's a D20H or D20 with SA0 high
                sa0 = sa0_high;
                if (device == device_auto)
                {
                    // use ID from WHO_AM_I register to determine device type
                    device = (id == D20H_WHO_ID) ? device_D20H : device_D20;
                }
            }
            // check SA0 low address unless SA0 was specified to be high
            else if (sa0 != sa0_high && (id = testReg(D20_SA0_LOW_ADDRESS, WHO_AM_I)) != TEST_REG_ERROR)
            {
                // device responds to address 1101010; it's a D20H or D20 with SA0 low
                sa0 = sa0_low;
                if (device == device_auto)
                {
                    // use ID from WHO_AM_I register to determine device type
                    device = (id == D20H_WHO_ID) ? device_D20H : device_D20;
                }
            }
        }

        // check for L3G4200D if device is still unidentified or was specified to be this type
        if (device == device_auto || device == device_4200D)
        {
            if (sa0 != sa0_low && testReg(L3G4200D_SA0_HIGH_ADDRESS, WHO_AM_I) == L3G4200D_WHO_ID)
            {
                // device responds to address 1101001; it's a 4200D with SA0 high
                device = device_4200D;
                sa0 = sa0_high;
            }
            else if (sa0 != sa0_high && testReg(L3G4200D_SA0_LOW_ADDRESS, WHO_AM_I) == L3G4200D_WHO_ID)
            {
                // device responds to address 1101000; it's a 4200D with SA0 low
                device = device_4200D;
                sa0 = sa0_low;
            }
        }

        // make sure device and SA0 were successfully detected; otherwise, indicate failure
        if (device == device_auto || sa0 == sa0_auto)
        {
            return false;
        }
    }

    _device = device;

    // set device address
    switch (device)
    {
    case device_D20H:
    case device_D20:
        address = (sa0 == sa0_high) ? D20_SA0_HIGH_ADDRESS : D20_SA0_LOW_ADDRESS;
        break;

    case device_4200D:
        address = (sa0 == sa0_high) ? L3G4200D_SA0_HIGH_ADDRESS : L3G4200D_SA0_LOW_ADDRESS;
        break;
    }

    _handle.setAddr(address);

    return true;
}

/*
Enables the L3G's gyro. Also:
- Sets gyro full scale (gain) to default power-on value of +/- 250 dps
  (specified as +/- 245 dps for L3GD20H).
- Selects 200 Hz ODR (output data rate). (Exact rate is specified as 189.4 Hz
  for L3GD20H and 190 Hz for L3GD20.)
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void L3G::enableDefault(void)
{
    if (_device == device_D20H)
    {
        // 0x00 = 0b00000000
        // Low_ODR = 0 (low speed ODR disabled)
        writeReg(LOW_ODR, 0x00);
    }

    // 0x00 = 0b00000000
    // FS = 00 (+/- 250 dps full scale)
    writeReg(CTRL_REG4, 0x00);

    // 0x6F = 0b01101111
    // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
    writeReg(CTRL_REG1, 0x6F);
}

// Writes a gyro register
void L3G::writeReg(uint8_t reg, uint8_t value)
{
    _handle.writeByte(reg, value);
}

// Reads a gyro register
uint8_t L3G::readReg(uint8_t reg)
{
    return _handle.readByte(reg);
}

// Reads the 3 gyro channels and stores them in vector g
bool L3G::read()
{
    bool ready = _handle.readBit(STATUS_REG, 3);
    if (ready)
    {
        uint8_t vals[6];
        if (_handle.readBytes(OUT_X_L | (1 << 7), 6, vals) < 0)
            return false;

        // combine high and low bytes
        g.x = (int16_t)(vals[1] << 8 | vals[0]);
        g.y = (int16_t)(vals[3] << 8 | vals[2]);
        g.z = (int16_t)(vals[5] << 8 | vals[4]);
    }
    return ready;
}

// Private Methods //////////////////////////////////////////////////////////////

int L3G::testReg(uint8_t address, regAddr reg)
{
    if (_handle.setAddr(address) < 0)
        return TEST_REG_ERROR;
    return _handle.readByte(reg);
}
