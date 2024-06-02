#include <Arduino.h>

// Include necessary libraries
#include <Wire.h>
#include <PulsePosition.h>

//debug serial print on/off
#define debug

// teensy pinconfiguration
int RecieverPin = 14; //PPM signal reciever
int LedGreenPin = 6; //LED Green Voltage battery
int LedRedPin = 5; //LED Red energy battery to low
int Motor1Pin = 1; //CCW Front Right
int Motor2Pin = 2; //CW Back Right
int Motor3Pin = 3; //CCW Back Left
int Motor4Pin = 4; //CW Front Left
int VoltageBatteryPin = 15; //Battery Voltage measuring 
int CurrentBatteryPin = 21; //Battery Current measuring

// Global variables for gyroscope readings
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

// Global variables for acceleration readings
float AccX_scaled,AccY_scaled,AccZ_scaled;
float CalibrationAccX,CalibrationAccY,CalibrationAccZ;

// Global variables for RC receiver inputs
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

// Global variables for battery status
float Voltage, Current, BatteryRemaining, BatteryAtStart;
float CurrentConsumed = 0;
float BatteryDefault = 1300;

// Timer variable for main loop
uint32_t LoopTimer;
uint32_t LoopTimer2;
uint32_t LoopTimer3;

// PID constants for roll, pitch, and yaw control
float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5;
float IRatePitch = IRateRoll;
float IRateYaw = 12;
float DRateRoll = 0.03;
float DRatePitch = DRateRoll;
float DRateYaw = 0;

// Array to store PID output and related variables
float PIDReturn[] = {0, 0, 0};

// Variables for PID reset
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

// Variables for desired rates
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw,  InputThrottle;

// Variables for pitch yaw roll
float ErrorRateRoll,ErrorRatePitch,ErrorRateYaw;
float InputRoll,InputPitch,InputYaw;

// Variables for Motorinputs
float MotorInput1,MotorInput2,MotorInput3,MotorInput4;

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

// Function to read battery voltage and current
void battery_voltage(void) 
{
  // Read voltage and current from analog pins
  Voltage = (float)analogRead(VoltageBatteryPin) / 62;
  Current = (float)analogRead(CurrentBatteryPin) * 0.089;
}

// Function to read signals from RC receiver
void read_receiver(void) {
  // Check the number of available channels
  ChannelNumber = ReceiverInput.available();  
  if (ChannelNumber > 0) {
    // Read each channel value
    for (int i = 1; i <= ChannelNumber; i++) {
      ReceiverValue[i - 1] = ReceiverInput.read(i);
    }
  }
}

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

  // Read and calculate acceleration readings
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

// Function to calculate PID output
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  // Calculate proportional, integral, and derivative terms
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;

  // Limit PID output
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;

  // Store PID output and related variables
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// Function to reset PID-related variables
void reset_pid(void) {
  PrevErrorRateRoll = 0;
  PrevErrorRatePitch = 0;
  PrevErrorRateYaw = 0;
  PrevItermRateRoll = 0;
  PrevItermRatePitch = 0;
  PrevItermRateYaw = 0;
}

void setup() {


  // Set pin modes and initial states
  pinMode(LedRedPin, OUTPUT);
  digitalWrite(LedRedPin, HIGH); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   

  // Initialize I2C communication and sensors
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // serial USB setup
  #ifdef debug
   Serial.begin(115200);
  #endif

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
  Wire.write(FILTER_BW_20); // Set low pass filter bandwidth to 256 Hz
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
  
  
  // Calibrate gyroscope readings
  calibrate_MPU();

  #ifdef debug 
    Serial.println("Calibration successfull");
    Serial.print("Calibrationnumber");
    Serial.print("\t");
    Serial.println(RateCalibrationNumber);
    Serial.print("Gyro");
    Serial.print("\t");
    Serial.print(RateCalibrationRoll,4);
    Serial.print("\t");
    Serial.print(RateCalibrationPitch,4);
    Serial.print("\t");
    Serial.println(RateCalibrationYaw,4);
    Serial.print("Accel");
    Serial.print("\t");
    Serial.print(CalibrationAccX,4);
    Serial.print("\t");
    Serial.print(CalibrationAccY,4);
    Serial.print("\t");
    Serial.println(1-CalibrationAccZ,4);
    Serial.println("Start loop");
    Serial.println();
    delay(1000);
  #endif

  // Configure PWM frequencies for motor control
  analogWriteFrequency(Motor1Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor2Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor3Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor4Pin, 250); //carrier frequency for PWM signal
  analogWriteResolution(12);  // nr bits resolution

  // Set initial battery status
  pinMode(LedGreenPin, OUTPUT);
  digitalWrite(LedGreenPin, HIGH);
  battery_voltage();
  if (Voltage > 8.3) { 
    digitalWrite(LedRedPin, LOW); 
    BatteryAtStart = BatteryDefault; 
  } else if (Voltage < 7.5) {
    BatteryAtStart = 30 / 100 * BatteryDefault;
  } else {
    digitalWrite(LedRedPin, LOW);
    BatteryAtStart = (82 * Voltage - 580) / 100 * BatteryDefault;
  }
  // Initialize RC receiver
  ReceiverInput.begin(RecieverPin);
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1200) {
    read_receiver();
    delay(4);
  }
  // Start loop timer
  LoopTimer = micros();
  LoopTimer2 = micros();
  LoopTimer3 = micros();
}

void loop() {
  // maintain loop rate
  if (micros() - LoopTimer > 4000){
    // Read gyroscope data
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;

    #ifdef debug
      if (micros() - LoopTimer3 > 400000){
      Serial.print("Gyro ");  
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
      Serial.print("D_R_roll");
      Serial.print("\t");
      Serial.print(DesiredRateRoll,0);
      Serial.print("\t");
      Serial.print("D_R_pitch");
      Serial.print("\t");
      Serial.print(DesiredRatePitch,0);
      Serial.print("\t");
      Serial.print("D_R_yaw");
      Serial.print("\t");
      Serial.print(DesiredRateYaw,0);
      Serial.print("\t");
      Serial.print("D_power");
      Serial.print("\t");
      Serial.print(InputThrottle,0);
      Serial.print("\t");
      Serial.print("E_R_roll");
      Serial.print("\t");
      Serial.print(ErrorRateRoll,0);
      Serial.print("\t");
      Serial.print("E_R_pitch");
      Serial.print("\t");
      Serial.print(ErrorRatePitch,0);
      Serial.print("\t");
      Serial.print("E_R_yaw");
      Serial.print("\t");
      Serial.print(ErrorRateYaw,0);
      Serial.print("\t");
      Serial.print("M4");
      Serial.print("\t");
      Serial.print(MotorInput4,0);
      Serial.print("\t");
      Serial.print("M3");
      Serial.print("\t");
      Serial.print(MotorInput3,0);
      Serial.print("\t");
      Serial.print("M2");
      Serial.print("\t");
      Serial.print(MotorInput2,0);
      Serial.print("\t");
      Serial.print("M1");
      Serial.print("\t");
      Serial.println(MotorInput1,0);
      LoopTimer3 = micros();
      }
    #endif  

    // Read receiver inputs
    read_receiver();
  
    // Calculate desired rates based on receiver inputs
    DesiredRateRoll = 0.15 * (ReceiverValue[0] - 1500);
    DesiredRatePitch = 0.15 * (ReceiverValue[1] - 1500);
    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

    // Calculate errors for PID control
    ErrorRateRoll = DesiredRateRoll - RateRoll;
    ErrorRatePitch = DesiredRatePitch - RatePitch;
    ErrorRateYaw = DesiredRateYaw - RateYaw;

    // Apply PID control for roll
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll = PIDReturn[0];
    PrevErrorRateRoll = PIDReturn[1];
    PrevItermRateRoll = PIDReturn[2];

    // Apply PID control for pitch
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch = PIDReturn[0];
    PrevErrorRatePitch = PIDReturn[1];
    PrevItermRatePitch = PIDReturn[2];

    // Apply PID control for yaw
    pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
    InputYaw = PIDReturn[0];
    PrevErrorRateYaw = PIDReturn[1];
    PrevItermRateYaw = PIDReturn[2];

    // Ensure throttle limits are respected
    if (InputThrottle > 1800) InputThrottle = 1800;

    // Calculate motor inputs
    MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
    MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);

    // Limit motor inputs
    if (MotorInput1 > 2048) MotorInput1 = 2048;
    if (MotorInput2 > 2048) MotorInput2 = 2048;
    if (MotorInput3 > 2048) MotorInput3 = 2048;
    if (MotorInput4 > 2048) MotorInput4 = 2048;

    // Set idle throttle level
    int ThrottleIdle = 1080;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    // Set cutoff throttle level if RC signal is lost
    int ThrottleCutOff = 1000;
    if (ReceiverValue[2] < 1020) {
      MotorInput1 = ThrottleCutOff; 
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff; 
      MotorInput4 = ThrottleCutOff;
      reset_pid();
    }

    // Send motor inputs to ESCs
    analogWrite(Motor1Pin, MotorInput1);
    analogWrite(Motor2Pin, MotorInput2);
    analogWrite(Motor3Pin, MotorInput3);
    analogWrite(Motor4Pin, MotorInput4);

    // Update battery status ; used amount of energy from battery based on current consumption
    battery_voltage();
    CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
    BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100;

    // Control LED based on battery level
    if (BatteryRemaining <= 30) digitalWrite(LedRedPin, HIGH);
    else digitalWrite(LedRedPin, LOW);

    if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
    }
  LoopTimer = micros();
  }
}
