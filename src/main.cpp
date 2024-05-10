#include <Arduino.h>

// Include necessary libraries
#include <Wire.h>

//debug serial print on/off
#define debug
//sensor fusion complementary filter on/off
//#define sensor_fusion

// MPU6050/9050 registers
  int device_address_MPU6050 = 0x68 ; 

  // Pre-defined ranges
  int ACCEL_RANGE_2G = 0x00;
  int ACCEL_RANGE_4G = 0x08;
  int ACCEL_RANGE_8G = 0x10;
  int ACCEL_RANGE_16G = 0x18;

  int GYRO_RANGE_250DEG = 0x00;
  int GYRO_RANGE_500DEG = 0x08;
  int GYRO_RANGE_1000DEG = 0x10;
  int GYRO_RANGE_2000DEG = 0x18;

  //Scale Modifiers
  int ACCEL_SCALE_MODIFIER_2G = 16384.0;
  int ACCEL_SCALE_MODIFIER_4G = 8192.0;
  int ACCEL_SCALE_MODIFIER_8G = 4096.0;
  int ACCEL_SCALE_MODIFIER_16G = 2048.0;

  int GYRO_SCALE_MODIFIER_250DEG = 131.0;
  int GYRO_SCALE_MODIFIER_500DEG = 65.5;
  int GYRO_SCALE_MODIFIER_1000DEG = 32.8;
  int GYRO_SCALE_MODIFIER_2000DEG = 16.4;

  int FILTER_BW_256=0x00;
  int FILTER_BW_188=0x01;
  int FILTER_BW_98=0x02;
  int FILTER_BW_42=0x03;
  int FILTER_BW_20=0x04;
  int FILTER_BW_10=0x05;
  int FILTER_BW_5=0x06;

  int I2C_MASTER_CTRL = 0x24;
  int USER_CTRL = 0x6A;
  int PWR_MGMT_1 = 0x6B;
  int PWR_MGMT_2 = 0x6C;

  int ACCEL_OUT = 0x3B;
  int ACCEL_XOUT0 = 0x3B;
  int ACCEL_YOUT0 = 0x3D;
  int ACCEL_ZOUT0 = 0x3F;

  int TEMP_OUT0 = 0x41;

  int GYRO_OUT = 0x43;
  int GYRO_XOUT0 = 0x43;
  int GYRO_YOUT0 = 0x45;
  int GYRO_ZOUT0 = 0x47;

  int ACCEL_CONFIG = 0x1C;
  int GYRO_CONFIG = 0x1B;
  int MPU_CONFIG = 0x1A;

  //AK8963 magneto registers
  int device_adress_AK8963 = 0x0C;
  int AK8963_CNTL   = 0x0A;
  int AK8963_XOUT_L = 0x03;
  int AK8963_ST1    = 0x02;
  int AK8963_ST2    = 0x09;
  int HXH           = 0x04; // HXL to HZH: Measurement Data
  int HYH           = 0x06;
  int HZH           = 0x08;
  int AK8963_ASAX   = 0x10;  // ASAX, ASAY, ASAZ: Sensitivity Adjustment values
  int mag_sens      = 4800.0; // magnetometer sensitivity: 4800 uT


// Global variables for gyroscope readings
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;
float roll_gyro, pitch_gyro, yaw_gyro;
float rollAngle , pitchAngle;

// Global variables for acceleration readings
float AccX_scaled,AccY_scaled,AccZ_scaled;
float CalibrationAccX,CalibrationAccY,CalibrationAccZ;

// Timer variable for main loop
uint32_t LoopTimer;
uint32_t LoopTimer2;
uint32_t LoopTimer3;
uint32_t current_time;
float time_difference;
uint32_t previous_time;

// Function to read gyroscope signals
void gyro_signals(void) {

  // Request gyroscope data from MPU6050 sensor
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(GYRO_OUT);
  Wire.endTransmission();
  Wire.requestFrom(device_address_MPU6050, 6);

  // Read and calculate gyroscope readings
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / GYRO_SCALE_MODIFIER_250DEG;
  RatePitch = (float)GyroY / GYRO_SCALE_MODIFIER_250DEG;
  RateYaw = (float)GyroZ / GYRO_SCALE_MODIFIER_250DEG;

  // Request acceleration data from MPU6050 sensor
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(ACCEL_OUT);
  Wire.endTransmission();
  Wire.requestFrom(device_address_MPU6050, 6);

  // Read and calculate gyroscope readings
  int16_t AccX = Wire.read() << 8 | Wire.read();
  int16_t AccY = Wire.read() << 8 | Wire.read();
  int16_t AccZ = Wire.read() << 8 | Wire.read();
  AccX_scaled = (float)AccX / ACCEL_SCALE_MODIFIER_2G;
  AccY_scaled = (float)AccY / ACCEL_SCALE_MODIFIER_2G;
  AccZ_scaled = (float)AccZ / ACCEL_SCALE_MODIFIER_2G;
}

  // Calibrate gyroscope and acceleration readings
 void calibrate_MPU(){
  RateCalibrationRoll = RateCalibrationPitch = RateCalibrationYaw = 0;
  CalibrationAccX = CalibrationAccY = CalibrationAccZ = 0;
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber ++) {
      gyro_signals();
      RateCalibrationRoll += RateRoll;
      RateCalibrationPitch += RatePitch;
      RateCalibrationYaw += RateYaw;
      CalibrationAccX += AccX_scaled;
      CalibrationAccY += AccY_scaled;
      CalibrationAccZ += AccZ_scaled;
      delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    CalibrationAccX /= 2000;
    CalibrationAccY /= 2000;
    CalibrationAccZ /= 2000;
 }

void setup() {
  #ifdef debug
   Serial.begin(115200);
  #endif

  // Set pin modes and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   

  // Initialize I2C communication and sensors
  Wire.setClock(400000); // Set I2C clock speed to 400 kHz
  Wire.begin(); // Initialize I2C communication
  delay(250); // Delay for sensor initialization

  #ifdef debug
    Serial.println("Begin transmission MPU6050"); 
  #endif

  // Begin transmission and configure Power Management 1 register for normal operation
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Configure for normal operation
  Wire.endTransmission();

  // Begin transmission and configure MPU Configuration register for low pass filter
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(MPU_CONFIG);
  Wire.write(FILTER_BW_256); // Set low pass filter bandwidth to 256 Hz
  Wire.endTransmission();

  // Begin transmission and configure Gyroscope Configuration register for sensitivity
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(GYRO_CONFIG);
  Wire.write(GYRO_RANGE_250DEG); // Set gyroscope range to ±250 degrees per second
  Wire.endTransmission();

  // Begin transmission and configure Acceleration configuration register for sensitivity
  Wire.beginTransmission(device_address_MPU6050);
  Wire.write(ACCEL_CONFIG);
  Wire.write(ACCEL_RANGE_2G); // Set acceleration full range to 2G 
  Wire.endTransmission();

  #ifdef debug 
    Serial.println("Init MPU6050 successfull");
    Serial.println("Start calibration Gyro and Acceleration sensor");
  #endif

  calibrate_MPU();

  #ifdef debug 
    Serial.println("Calibration successfull");
    Serial.print("Gyro");
    Serial.print("\t");
    Serial.print(RateCalibrationRoll);
    Serial.print("\t");
    Serial.print(RateCalibrationPitch);
    Serial.print("\t");
    Serial.println(RateCalibrationYaw);
    Serial.print("Accel");
    Serial.print("\t");
    Serial.print(CalibrationAccX);
    Serial.print("\t");
    Serial.print(CalibrationAccY);
    Serial.print("\t");
    Serial.println(1-CalibrationAccZ);
    Serial.println("Start loop");
    Serial.println();
    delay(1000);
  #endif

  // Start loop timer
  LoopTimer = micros();
  LoopTimer2 = micros();
  LoopTimer3 = micros();
  previous_time = micros();
  roll_gyro = pitch_gyro = yaw_gyro = 0;
}

void loop() {
  if (micros() - LoopTimer > 10000){
    // Read gyroscope data
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
    AccX_scaled -= CalibrationAccX;
    AccY_scaled -= CalibrationAccY;
    AccZ_scaled = AccZ_scaled+(1-CalibrationAccZ);
    
    current_time = micros();
    time_difference = (current_time - previous_time)/1000000.0;

    // Calculate change in orientation using gyroscope data
    roll_gyro += RateRoll * time_difference;
    pitch_gyro += RatePitch * time_difference;
    yaw_gyro += RateYaw * time_difference;
    
    // Calculate roll angle in degrees
    rollAngle = atan2(AccY_scaled, sqrt(AccX_scaled*AccX_scaled + AccZ_scaled * AccZ_scaled)) * 180.0 / PI;

    // Calculate pitch angle in degrees
    pitchAngle = atan2(-AccX_scaled, sqrt(AccY_scaled * AccY_scaled + AccZ_scaled * AccZ_scaled)) * 180.0 / PI;
    // Reset loop timer for the next iteration

    // sensor fusion with complementary filter gyro and accelerator sensor
    #ifdef sensor_fusion 
      roll_gyro = 0.98 * roll_gyro + 0.02 * rollAngle;
      pitch_gyro = 0.98 * pitch_gyro + 0.02 * pitchAngle;
    #endif

    LoopTimer = micros();
    previous_time = current_time;
  }

  #ifdef debug
    if (micros() - LoopTimer3 > 400000){
    Serial.print("Rate Roll");
    Serial.print("\t");
    Serial.print(RateRoll,0);
    Serial.print("\t");
    Serial.print("Pitch");
    Serial.print("\t");
    Serial.print(RatePitch,0);
    Serial.print("\t");
    Serial.print("Yaw");
    Serial.print("\t");
    Serial.print(RateYaw,0);
    Serial.print("\t");
    Serial.print("Roll_gyro");
    Serial.print("\t");
    Serial.print(roll_gyro,0);
    Serial.print("\t");
    Serial.print("Pitch_gyro");
    Serial.print("\t");
    Serial.print(pitch_gyro,0);
    Serial.print("\t");
    Serial.print("Yaw_gyro");
    Serial.print("\t");
    Serial.print(yaw_gyro,0);
    Serial.print("\t");
    Serial.print("Acc X");
    Serial.print("\t");
    Serial.print(AccX_scaled);
    Serial.print("\t");
    Serial.print("Y");
    Serial.print("\t");
    Serial.print(AccY_scaled);
    Serial.print("\t");
    Serial.print("Z");
    Serial.print("\t");
    Serial.print(AccZ_scaled);
    Serial.print("\t");
    Serial.print("Roll_Acc");
    Serial.print("\t");
    Serial.print(rollAngle,0);
    Serial.print("\t");
    Serial.print("Pitch_Acc");
    Serial.print("\t");
    Serial.println(pitchAngle,0);
    LoopTimer3 = micros();
    }
  #endif


  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }
}
