#include <iostream>
#include <cmath>
#include <unistd.h>

#include "i2c.h"
#include "Bmp085Driver.h"

const uint8_t BMP085_RA_AC1_H =   0xAA;    // AC1_H
const uint8_t BMP085_RA_AC1_L =   0xAB;    // AC1_L
const uint8_t BMP085_RA_AC2_H =   0xAC;    // AC2_H
const uint8_t BMP085_RA_AC2_L =   0xAD;    //AC2_L
const uint8_t BMP085_RA_AC3_H =   0xAE;    // AC3_H
const uint8_t BMP085_RA_AC3_L =   0xAF;    // AC3_L
const uint8_t BMP085_RA_AC4_H =   0xB0;    // AC4_H
const uint8_t BMP085_RA_AC4_L =   0xB1;    // AC4_L
const uint8_t BMP085_RA_AC5_H =   0xB2;    // AC5_H
const uint8_t BMP085_RA_AC5_L =   0xB3;    // AC5_L
const uint8_t BMP085_RA_AC6_H =   0xB4;    // AC6_H
const uint8_t BMP085_RA_AC6_L =   0xB5;    // AC6_L
const uint8_t BMP085_RA_B1_H =    0xB6;    // B1_H
const uint8_t BMP085_RA_B1_L =    0xB7;    // B1_L
const uint8_t BMP085_RA_B2_H =    0xB8;    // B2_H
const uint8_t BMP085_RA_B2_L =    0xB9;    // B2_L
const uint8_t BMP085_RA_MB_H =    0xBA;    // MB_H
const uint8_t BMP085_RA_MB_L =    0xBB;    // MB_L
const uint8_t BMP085_RA_MC_H =    0xBC;    // MC_H
const uint8_t BMP085_RA_MC_L =    0xBD;    // MC_L
const uint8_t BMP085_RA_MD_H =    0xBE;    // MD_H
const uint8_t BMP085_RA_MD_L =    0xBF;    // MD_L
const uint8_t BMP085_RA_CONTROL = 0xF4;    // CONTROL
const uint8_t BMP085_RA_MSB =     0xF6;    // MSB
const uint8_t BMP085_RA_LSB =     0xF7;    // LSB
const uint8_t BMP085_RA_XLSB =    0xF8;    // XLSB

Bmp085Driver::Bmp085Driver(I2CInterface& interface) : _handle(interface) { }

/**
 * Prepare device for normal usage.
 */
void Bmp085Driver::initialize() {
    // load sensor's calibration constants
    loadCalibration();
}

/**
 * Verify the device is connected and available.
 */
bool Bmp085Driver::testConnection() {
    // TODO
}

/* calibration register methods */

void Bmp085Driver::loadCalibration() {
    uint8_t buf2[22];
    _handle.readBytes(BMP085_RA_AC1_H, 22, buf2);
    ac1 = ((int16_t)buf2[0] << 8) + buf2[1];
    ac2 = ((int16_t)buf2[2] << 8) + buf2[3];
    ac3 = ((int16_t)buf2[4] << 8) + buf2[5];
    ac4 = ((uint16_t)buf2[6] << 8) + buf2[7];
    ac5 = ((uint16_t)buf2[8] << 8) + buf2[9];
    ac6 = ((uint16_t)buf2[10] << 8) + buf2[11];
    b1 = ((int16_t)buf2[12] << 8) + buf2[13];
    b2 = ((int16_t)buf2[14] << 8) + buf2[15];
    mb = ((int16_t)buf2[16] << 8) + buf2[17];
    mc = ((int16_t)buf2[18] << 8) + buf2[19];
    md = ((int16_t)buf2[20] << 8) + buf2[21];
    calibrationLoaded = true;
}

/* control register methods */

uint8_t Bmp085Driver::getControl() {
    return _handle.readByte(BMP085_RA_CONTROL);
}
void Bmp085Driver::setControl(uint8_t value) {
    _handle.writeByte(BMP085_RA_CONTROL, value);
    measureMode = value;
}

/* measurement register methods */

uint16_t Bmp085Driver::getMeasurement2() {
    _handle.readBytes(BMP085_RA_MSB, 2, _buffer);
    return ((uint16_t)_buffer[0] << 8) + _buffer[1];
}
uint32_t Bmp085Driver::getMeasurement3() {
    _handle.readBytes(BMP085_RA_MSB, 3, _buffer);
    return ((uint32_t)_buffer[0] << 16) + ((uint16_t)_buffer[1] << 8) + _buffer[2];
}
uint8_t Bmp085Driver::getMeasureDelayMilliseconds(uint8_t mode) {
    if (mode == 0) mode = measureMode;
    if (measureMode == 0x2E) return 5;
    else if (measureMode == 0x34) return 5;
    else if (measureMode == 0x74) return 8;
    else if (measureMode == 0xB4) return 14;
    else if (measureMode == 0xF4) return 26;
    return 0; // invalid mode
}
uint16_t Bmp085Driver::getMeasureDelayMicroseconds(uint8_t mode) {
    if (mode == 0) mode = measureMode;
    if (measureMode == 0x2E) return 4500;
    else if (measureMode == 0x34) return 4500;
    else if (measureMode == 0x74) return 7500;
    else if (measureMode == 0xB4) return 13500;
    else if (measureMode == 0xF4) return 25500;
    return 0; // invalid mode
}

uint16_t Bmp085Driver::getRawTemperature() {
    if (measureMode == 0x2E) return getMeasurement2();
    return 0; // wrong measurement mode for temperature request
}

double Bmp085Driver::getTemperatureC() {
    /*
    Datasheet formula:
        UT = raw temperature
        X1 = (UT - AC6) * AC5 / 2^15
        X2 = MC * 2^11 / (X1 + MD)
        B5 = X1 + X2
        T = (B5 + 8) / 2^4
    */
    int32_t ut = getRawTemperature();
    int32_t x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
    int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
    b5 = x1 + x2;
    return ((b5 + 8) >> 4) / 10.0;
}

double Bmp085Driver::getTemperatureF() {
    return getTemperatureC() * 9.0f / 5.0f + 32;
}

uint32_t Bmp085Driver::getRawPressure() {
    if (measureMode & 0x34) return getMeasurement3() >> (8 - ((measureMode & 0xC0) >> 6));
    return 0; // wrong measurement mode for pressure request
}

double Bmp085Driver::getPressure() {
    /*
    Datasheet forumla
        UP = raw pressure
        B6 = B5 - 4000
        X1 = (B2 * (B6 * B6 / 2^12)) / 2^11
        X2 = AC2 * B6 / 2^11
        X3 = X1 + X2
        B3 = ((AC1 * 4 + X3) << oss + 2) / 4
        X1 = AC3 * B6 / 2^13
        X2 = (B1 * (B6 * B6 / 2^12)) / 2^16
        X3 = ((X1 + X2) + 2) / 2^2
        B4 = AC4 * (unsigned long)(X3 + 32768) / 2^15
        B7 = ((unsigned long)UP - B3) * (50000 >> oss)
        if (B7 < 0x80000000) { p = (B7 * 2) / B4 }
        else { p = (B7 / B4) * 2 }
        X1 = (p / 2^8) * (p / 2^8)
        X1 = (X1 * 3038) / 2^16
        X2 = (-7357 * p) / 2^16
        p = p + (X1 + X2 + 3791) / 2^4
    */
    uint32_t up = getRawPressure();
    uint8_t oss = (measureMode & 0xC0) >> 6;
    int32_t p;
    int32_t b6 = b5 - 4000;
    int32_t x1 = ((int32_t)b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = ((int32_t)ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)ac1 * 4 + x3) << oss) + 2) >> 2;
    x1 = ((int32_t)ac3 * b6) >> 13;
    x2 = ((int32_t)b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (uint32_t)(50000UL >> oss);
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    return p + ((x1 + x2 + (int32_t)3791) >> 4);
}

double Bmp085Driver::getAltitude(double pressure, double seaLevelPressure) {
    return 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

