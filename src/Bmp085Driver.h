#ifndef BMP085DRIVER_H
#define BMP085DRIVER_H

#include "i2c.h"

#define BMP085_ADDRESS 0x77 // this device only has one address

const uint8_t BMP085_MODE_TEMPERATURE = 0x2E;
const uint8_t BMP085_MODE_PRESSURE_0 =  0x34;
const uint8_t BMP085_MODE_PRESSURE_1 =  0x74;
const uint8_t BMP085_MODE_PRESSURE_2 =  0xB4;
const uint8_t BMP085_MODE_PRESSURE_3 =  0xF4;

class Bmp085Driver {
    public:
        Bmp085Driver(I2CInterface& interface);

        void initialize();
        bool testConnection();

        // CONTROL register methods
        uint8_t     getControl();
        void        setControl(uint8_t value);

        // MEASURE register methods
        uint16_t    getMeasurement2(); // 16-bit data
        uint32_t    getMeasurement3(); // 24-bit data
        uint8_t     getMeasureDelayMilliseconds(uint8_t mode=0);
        uint16_t    getMeasureDelayMicroseconds(uint8_t mode=0);

        // convenience methods
        void        loadCalibration();
        uint16_t    getRawTemperature();
        float       getTemperatureC();
        float       getTemperatureF();
        uint32_t    getRawPressure();
        float       getPressure();
        float       getAltitude(float pressure, float seaLevelPressure=101325);

    private:
        I2CInterface& _handle;
        uint8_t _buffer[3];

        bool calibrationLoaded;
        int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
        uint16_t ac4, ac5, ac6;
        int32_t b5;
        uint8_t measureMode;
};

#endif // BMP085DRIVER_H
