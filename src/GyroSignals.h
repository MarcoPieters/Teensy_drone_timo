// GyroSignals.h
#ifndef GYRO_SIGNALS_H
#define GYRO_SIGNALS_H

#include <Wire.h>

class GyroSignals {
public:
    GyroSignals();
    void init();
    void calibrate(float &RateCalibrationRoll, float &RateCalibrationPitch, float &RateCalibrationYaw,
                            float &CalibrationAccX, float &CalibrationAccY, float &CalibrationAccZ, 
                            int calibrationIterations = 2000);
    void readGyroData(float &rateRoll, float &ratePitch, float &rateYaw);
    void readAccelData(float &accXScaled, float &accYScaled, float &accZScaled);
                

private:
    
    float RateCalibrationRoll;
    float RateCalibrationPitch;
    float RateCalibrationYaw;
    float CalibrationAccX;
    float CalibrationAccY;
    float CalibrationAccZ;
   
    static const int device_address_MPU6050 = 0x68;  // I2C address

    static const int MPU_CONFIG = 0x1A;         // register init 26
    static const int FILTER_BW_256=0x00;
    static const int FILTER_BW_188=0x01;
    static const int FILTER_BW_98=0x02;
    static const int FILTER_BW_42=0x03;
    static const int FILTER_BW_20=0x04;
    static const int FILTER_BW_10=0x05;
    static const int FILTER_BW_5=0x06;

    static const int GYRO_CONFIG = 0x1B;        // register init 27
    static const int GYRO_RANGE_250DEG = 0x00;
    static const int GYRO_RANGE_500DEG = 0x08;
    static const int GYRO_RANGE_1000DEG = 0x10;
    static const int GYRO_RANGE_2000DEG = 0x18;
    static const int GYRO_SCALE_MODIFIER_250DEG = 131.0;
    static const int GYRO_SCALE_MODIFIER_500DEG = 65.5;
    static const int GYRO_SCALE_MODIFIER_1000DEG = 32.8;
    static const int GYRO_SCALE_MODIFIER_2000DEG = 16.4;

    static const int ACCEL_CONFIG = 0x1C;       // register init 28
    static const int ACCEL_RANGE_2G = 0x00;
    static const int ACCEL_RANGE_4G = 0x08;
    static const int ACCEL_RANGE_8G = 0x10;
    static const int ACCEL_RANGE_16G = 0x18;

    static const int ACCEL_SCALE_MODIFIER_2G = 16384.0;
    static const int ACCEL_SCALE_MODIFIER_4G = 8192.0;
    static const int ACCEL_SCALE_MODIFIER_8G = 4096.0;
    static const int ACCEL_SCALE_MODIFIER_16G = 2048.0;
    
    static const int I2C_MASTER_CTRL = 0x24;    // register init 36

    static const int ACCEL_OUT = 0x3B;          // register read 59
    static const int ACCEL_XOUT0 = 0x3B;        // register read 59
    static const int ACCEL_YOUT0 = 0x3D;        // register read 61
    static const int ACCEL_ZOUT0 = 0x3F;        // register read 63

    static const int TEMP_OUT0 = 0x41;          // register read 65

    static const int GYRO_OUT = 0x43;           // register read 67
    static const int GYRO_XOUT0 = 0x43;         // register read 67
    static const int GYRO_YOUT0 = 0x45;         // register read 69
    static const int GYRO_ZOUT0 = 0x47;         // register read 71

    static const int USER_CTRL = 0x6A;          // register init 106
    static const int PWR_MGMT_1 = 0x6B;         // register init 107
    static const int PWR_MGMT_2 = 0x6C;         // register init 108
};

#endif // GYRO_SIGNALS_H
