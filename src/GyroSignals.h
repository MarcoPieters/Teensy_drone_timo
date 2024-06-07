// GyroSignals.h
#ifndef GYRO_SIGNALS_H
#define GYRO_SIGNALS_H

#include <Wire.h>

class GyroSignals {
public:
    GyroSignals(int deviceAddress);
    void init();
    void readSignals(float &rateRoll, float &ratePitch, float &rateYaw, 
                     float &accXScaled, float &accYScaled, float &accZScaled);
    void calibrate(float &RateCalibrationRoll, float &RateCalibrationPitch, float &RateCalibrationYaw,
                   float &CalibrationAccX, float &CalibrationAccY, float &CalibrationAccZ);                     

private:
    int deviceAddress;
    float RateCalibrationRoll;
    float RateCalibrationPitch;
    float RateCalibrationYaw;
    float CalibrationAccX;
    float CalibrationAccY;
    float CalibrationAccZ;

    void readGyroData(float &rateRoll, float &ratePitch, float &rateYaw);
    void readAccelData(float &accXScaled, float &accYScaled, float &accZScaled);

    static const int ACCEL_RANGE_2G = 0x00;
    static const int ACCEL_RANGE_4G = 0x08;
    static const int ACCEL_RANGE_8G = 0x10;
    static const int ACCEL_RANGE_16G = 0x18;

    static const int GYRO_RANGE_250DEG = 0x00;
    static const int GYRO_RANGE_500DEG = 0x08;
    static const int GYRO_RANGE_1000DEG = 0x10;
    static const int GYRO_RANGE_2000DEG = 0x18;

    static const int ACCEL_SCALE_MODIFIER_2G = 16384.0;
    static const int ACCEL_SCALE_MODIFIER_4G = 8192.0;
    static const int ACCEL_SCALE_MODIFIER_8G = 4096.0;
    static const int ACCEL_SCALE_MODIFIER_16G = 2048.0;

    static const int GYRO_SCALE_MODIFIER_250DEG = 131.0;
    static const int GYRO_SCALE_MODIFIER_500DEG = 65.5;
    static const int GYRO_SCALE_MODIFIER_1000DEG = 32.8;
    static const int GYRO_SCALE_MODIFIER_2000DEG = 16.4;

    static const int FILTER_BW_256=0x00;
    static const int FILTER_BW_188=0x01;
    static const int FILTER_BW_98=0x02;
    static const int FILTER_BW_42=0x03;
    static const int FILTER_BW_20=0x04;
    static const int FILTER_BW_10=0x05;
    static const int FILTER_BW_5=0x06;

    static const int I2C_MASTER_CTRL = 0x24;
    static const int USER_CTRL = 0x6A;
    static const int PWR_MGMT_1 = 0x6B;
    static const int PWR_MGMT_2 = 0x6C;

    static const int ACCEL_OUT = 0x3B;
    static const int ACCEL_XOUT0 = 0x3B;
    static const int ACCEL_YOUT0 = 0x3D;
    static const int ACCEL_ZOUT0 = 0x3F;

    static const int TEMP_OUT0 = 0x41;

    static const int GYRO_OUT = 0x43;
    static const int GYRO_XOUT0 = 0x43;
    static const int GYRO_YOUT0 = 0x45;
    static const int GYRO_ZOUT0 = 0x47;

    static const int ACCEL_CONFIG = 0x1C;
    static const int GYRO_CONFIG = 0x1B;
    static const int MPU_CONFIG = 0x1A;
};

#endif // GYRO_SIGNALS_H
