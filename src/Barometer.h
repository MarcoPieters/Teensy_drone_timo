#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>

struct BaroData {
    float pressure;
    float cTemp;
};

class Barometer {
public:
    Barometer(int address);
    void begin();
    BaroData readData();
    float calculateAltitude(float pressure);
    float getInitialPressure();

private:
    int _address;
    float _initialPressure;
    const int _ctrl_meas = 0xF4;
    const int _config = 0xF5;
    void readCoefficients();
    void readRawData(uint32_t &adc_p, uint32_t &adc_t);
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
};

#endif
