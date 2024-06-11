#include "Barometer.h"

Barometer::Barometer(int address) : _address(address) {}

void Barometer::begin() {
    
    #ifdef debug
    Serial.println("Begin transmission BMP280");
    #endif

    Wire.beginTransmission(_address);
    Wire.write(_ctrl_meas);
    Wire.write(0b10101011); // Configure for normal operation
    Wire.endTransmission();

    Wire.beginTransmission(_address);
    Wire.write(_config);
    Wire.write(0b10101100); // Configure for normal operation
    Wire.endTransmission();

    delay(500);

    #ifdef debug
    Serial.println("Init BMP280 successful");
    #endif

    readCoefficients();
    _initialPressure = readData().pressure;
}

BaroData Barometer::readData() {
    BaroData data;
    uint32_t adc_p, adc_t;

    readRawData(adc_p, adc_t);

    // Temperature offset calculations
    float var1 = (double(adc_t) / 16384.0 - double(dig_T1) / 1024.0) * double(dig_T2);
    float var2 = ((double(adc_t) / 131072.0 - double(dig_T1) / 8192.0) * (double(adc_t) / 131072.0 - double(dig_T1) / 8192.0)) * double(dig_T3);
    float t_fine = var1 + var2;
    data.cTemp = t_fine / 5120.0;

    // Pressure offset calculations
    float var3 = (t_fine / 2.0) - 64000.0;
    float var4 = var3 * var3 * double(dig_P6) / 32768.0;
    var4 = var4 + var3 * double(dig_P5) * 2.0;
    var4 = (var4 / 4.0) + (double(dig_P4) * 65536.0);
    var3 = (double(dig_P3) * var3 * var3 / 524288.0 + double(dig_P2) * var3) / 524288.0;
    var3 = (1.0 + var3 / 32768.0) * (dig_P1);
    float p = 1048576.0 - adc_p;
    p = (p - (var4 / 4096.0)) * 6250.0 / var3;
    float var5 = double(dig_P9) * p * p / 2147483648.0;
    float var6 = p * double(dig_P8) / 32768.0;
    data.pressure = (p + (var5 + var6 + double(dig_P7)) / 16.0) / 100.0;

    return data;
}

float Barometer::calculateAltitude(float pressure) {
    const float seaLevelPressure = 101325.0; // Sea level pressure in pascals
    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903)) * 100; // altitude in cm
    return altitude;
}

float Barometer::getInitialPressure() {
    return _initialPressure;
}

void Barometer::readCoefficients() {
    Wire.beginTransmission(_address);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(_address, 24);

    byte b1[24];
    for(int i = 0; i < 24; i++) {
        b1[i] = Wire.read();
    }

    dig_T1 = (b1[1] << 8) | b1[0];
    dig_T2 = (b1[3] << 8) | b1[2];
    dig_T3 = (b1[5] << 8) | b1[4];
    dig_P1 = (b1[7] << 8) | b1[6];
    dig_P2 = (b1[9] << 8) | b1[8];
    dig_P3 = (b1[11] << 8) | b1[10];
    dig_P4 = (b1[13] << 8) | b1[12];
    dig_P5 = (b1[15] << 8) | b1[14];
    dig_P6 = (b1[17] << 8) | b1[16];
    dig_P7 = (b1[19] << 8) | b1[18];
    dig_P8 = (b1[21] << 8) | b1[20];
    dig_P9 = (b1[23] << 8) | b1[22];
}

void Barometer::readRawData(uint32_t &adc_p, uint32_t &adc_t) {
    Wire.beginTransmission(_address);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(_address, 8);

    byte data_buffer[8];
    for(int i = 0; i < 8; i++) {
        data_buffer[i] = Wire.read();
    }

    adc_p = ((uint32_t)data_buffer[0] << 16) | ((uint32_t)data_buffer[1] << 8) | (uint32_t)(data_buffer[2] & 0xF0);
    adc_p /= 16;
    adc_t = ((uint32_t)data_buffer[3] << 16) | ((uint32_t)data_buffer[4] << 8) | (uint32_t)(data_buffer[5] & 0xF0);
    adc_t /= 16;
}
