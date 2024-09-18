// GyroSignals.cpp
#include "GyroSignals.h"

GyroSignals::GyroSignals() 
        {}

void GyroSignals::init() {
    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(PWR_MGMT_1); // PWR_MGMT_1
    Wire.write(0x00); // Wake up the MPU6050
    Wire.endTransmission();

    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(MPU_CONFIG); // MPU_CONFIG
    Wire.write(FILTER_BW_20); // Set low pass filter bandwidth to 20 Hz
    Wire.endTransmission();

    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(GYRO_CONFIG); // GYRO_CONFIG
    Wire.write(GYRO_RANGE_250DEG); // Set gyroscope range to ±250 degrees per second
    Wire.endTransmission();

    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(ACCEL_CONFIG); // ACCEL_CONFIG
    Wire.write(ACCEL_RANGE_2G); // Set acceleration full range to 2G
    Wire.endTransmission();
}

void GyroSignals::calibrate(float &RateCalibrationRoll, float &RateCalibrationPitch, float &RateCalibrationYaw,
                            float &CalibrationAccX, float &CalibrationAccY, float &CalibrationAccZ, 
                            int calibrationIterations) {

    RateCalibrationRoll = 0;
    RateCalibrationPitch = 0;
    RateCalibrationYaw = 0;
    CalibrationAccX = 0;
    CalibrationAccY = 0;
    CalibrationAccZ = 0;

    for (int i = 0; i < calibrationIterations; i++) {
        float RateRoll, RatePitch, RateYaw, AccX_scaled, AccY_scaled, AccZ_scaled;
        readGyroData(RateRoll, RatePitch, RateYaw);
        readAccelData(AccX_scaled, AccY_scaled, AccZ_scaled);
        // readSignals(RateRoll, RatePitch, RateYaw, AccX_scaled, AccY_scaled, AccZ_scaled);
        
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        CalibrationAccX += AccX_scaled;
        CalibrationAccY += AccY_scaled;
        CalibrationAccZ += AccZ_scaled;

        delay(1);
    }

    RateCalibrationRoll /= calibrationIterations;
    RateCalibrationPitch /= calibrationIterations;
    RateCalibrationYaw /= calibrationIterations;

    CalibrationAccX /= calibrationIterations;
    CalibrationAccY /= calibrationIterations;
    CalibrationAccZ /= calibrationIterations;
}


void GyroSignals::readGyroData(float &rateRoll, float &ratePitch, float &rateYaw) {
    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(GYRO_OUT);
    Wire.endTransmission();
    Wire.requestFrom(device_address_MPU6050, 6);

    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();
    rateRoll = (float)gyroX / GYRO_SCALE_MODIFIER_250DEG;
    ratePitch = (float)gyroY / GYRO_SCALE_MODIFIER_250DEG;
    rateYaw = (float)gyroZ / GYRO_SCALE_MODIFIER_250DEG;
}

void GyroSignals::readAccelData(float &accXScaled, float &accYScaled, float &accZScaled) {
    Wire.beginTransmission(device_address_MPU6050);
    Wire.write(ACCEL_OUT);
    Wire.endTransmission();
    Wire.requestFrom(device_address_MPU6050, 6);

    int16_t accX = Wire.read() << 8 | Wire.read();
    int16_t accY = Wire.read() << 8 | Wire.read();
    int16_t accZ = Wire.read() << 8 | Wire.read();
    accXScaled = (float)accX / ACCEL_SCALE_MODIFIER_2G;
    accYScaled = (float)accY / ACCEL_SCALE_MODIFIER_2G;
    accZScaled = (float)accZ / ACCEL_SCALE_MODIFIER_2G;
}
