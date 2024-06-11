#include <Arduino.h>

// Include necessary libraries
#include <Wire.h>
#include <PulsePosition.h>
#include "wiring.h"
#include "GyroSignals.h"


//debug serial print on/off
#define debug
#define debug_text
//#define debug_graph

#define sensor_fusion

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

int steering_manner = 2; // 1 for rate ; 2 for angle

// Global variables for gyroscope readings
float RatePitch, RateRoll, RateYaw;
float CalibrationGyroPitch, CalibratioGyroRoll, CalibrationGyroYaw;
int RateCalibrationNumber;
float roll_angle_gyro, pitch_angle_gyro, yaw_angle_gyro;
float roll_angle_gyro_fusion, pitch_angle_gyro_fusion;
float roll_angle_acc , pitch_angle_acc;

// Global variables for acceleration readings
float AccX,AccY,AccZ;
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
uint32_t current_time;
float time_difference;
uint32_t previous_time;

// PID constants for roll, pitch, and yaw control
float PRateRoll = 0.6;  //0.6
float PRatePitch = PRateRoll;
float PRateYaw = 2;
float IRateRoll = 3.5; // 3.5
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
float PrevErrorAngleRoll, PrevErrorAnglePitch, PrevErrorAngleYaw;
float PrevItermAngleRoll, PrevItermAnglePitch, PrevItermAngleYaw;

// Variables for desired rates
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw,  InputThrottle;
float DesiredAngleRoll, DesiredAnglePitch, DesiredAngleYaw;

// Variables for pitch yaw roll
float ErrorRateRoll,ErrorRatePitch,ErrorRateYaw;
float ErrorAngleRoll,ErrorAnglePitch,ErrorAngleYaw;
float InputRoll,InputPitch,InputYaw;

// Variables for Motorinputs
float MotorInput1,MotorInput2,MotorInput3,MotorInput4;

// Declare Class GyroSignals
GyroSignals gyroSignals;

// Function to read battery voltage and current
void battery_voltage(void) 
{
  // Read voltage and current from analog pins
  Voltage = (float)analogRead(VoltageBatteryPin) / 40.3;
  Current = (float)analogRead(CurrentBatteryPin) / 2.0 ;  //0.089 scale current to amps 1/75 for Mateksys FCHUB-12S fullrange 440A max 3,3V.
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

  PrevErrorAngleRoll = 0;
  PrevErrorAnglePitch = 0;
  PrevErrorAngleYaw = 0;
  PrevItermAngleRoll = 0;
  PrevItermAnglePitch = 0;
  PrevItermAngleYaw = 0;
}

void setup() {
  // Set pin modes and initial states
  pinMode(LedRedPin, OUTPUT);
  digitalWrite(LedRedPin, HIGH); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   

  // serial USB setup
  #ifdef debug
   Serial.begin(115200);
  #endif

  #ifdef debug
    Serial.println("Begin initializing I2C"); 
  #endif

  // Initialize I2C communication and sensors
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  #ifdef debug
    Serial.println("Begin initializing MPU6050"); 
  #endif
  
  // Initialize Gyro and Accelleration sensor
  gyroSignals.init();

  #ifdef debug 
    Serial.println("Init MPU6050 successfull");
    Serial.println("Start calibration Gyro and Acceleration sensor");
  #endif
    
  // Calibrate gyroscope readings
  gyroSignals.calibrate(CalibratioGyroRoll, CalibrationGyroPitch, CalibrationGyroYaw,
                             CalibrationAccX, CalibrationAccY, CalibrationAccZ);

  #ifdef debug 
    Serial.println();
    Serial.println("Calibration successfull");
    Serial.print("Calibrationsamples");
    Serial.print("\t");
    Serial.println(RateCalibrationNumber);
    Serial.println("Correction factors sensors after calibrating");
    Serial.print("  Gyro");
    Serial.print("\t");
    Serial.print(CalibratioGyroRoll,4);
    Serial.print("\t");
    Serial.print(CalibrationGyroPitch,4);
    Serial.print("\t");
    Serial.println(CalibrationGyroYaw,4);
    Serial.print("  Accel");
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
  if (Voltage > 14.8) { 
    digitalWrite(LedRedPin, LOW); 
    BatteryAtStart = BatteryDefault; 
  } else if (Voltage < 14.0) {
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

  switch (steering_manner)
  {
  case 1:
    PRateRoll = 0.6;  //0.6
    PRatePitch = PRateRoll;
    PRateYaw = 2;
    IRateRoll = 3.5; // 3.5
    IRatePitch = IRateRoll;
    IRateYaw = 12;
    DRateRoll = 0.03;
    DRatePitch = DRateRoll;
    DRateYaw = 0;
    break;

  case 2:
    PRateRoll = 2; // 5 :resonantie op 10
    PRatePitch = 2; //resonantie op 10
    PRateYaw = 2;
    IRateRoll = 0; // 3.5
    IRatePitch = IRateRoll;
    IRateYaw = 0;
    DRateRoll = 0.03;
    DRatePitch = DRateRoll;
    DRateYaw = 0;
    break;

  default:
    break;
  }

  // Start loop timer
  LoopTimer = micros();
  LoopTimer2 = micros();
  LoopTimer3 = micros();
  previous_time = micros();
}


void loop()
{
  // maintain loop rate
  if (micros() - LoopTimer > 4000){
    // Read gyroscope data
    gyroSignals.readSignals(RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ);
    // Gyro ACC correction with calibrationfactors
    RateRoll -= CalibratioGyroRoll;  
    RatePitch -= CalibrationGyroPitch; 
    RateYaw -= CalibrationGyroYaw; 
    AccX -= CalibrationAccX;
    AccY -= CalibrationAccY;
    AccZ = AccZ+(1-CalibrationAccZ);
    
    current_time = micros();
    time_difference = (current_time - previous_time)/1000000.0; // scale microseconds to seconds

    // Calculate change in orientation using gyroscope data
    roll_angle_gyro += RateRoll * time_difference;
    pitch_angle_gyro += RatePitch * time_difference;
    yaw_angle_gyro += RateYaw * time_difference;
    
    roll_angle_gyro_fusion += RateRoll * time_difference;
    pitch_angle_gyro_fusion += RatePitch * time_difference;

    // Calculate roll angle in degrees
    roll_angle_acc = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / PI;

    // Calculate pitch angle in degrees
    pitch_angle_acc = atan2(-AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / PI;
    
    // sensor fusion with complementary filter gyro and accelerator sensor
    #ifdef sensor_fusion 
      roll_angle_gyro_fusion = 0.98 * roll_angle_gyro_fusion + 0.02 * roll_angle_acc;
      pitch_angle_gyro_fusion = 0.98 * pitch_angle_gyro_fusion + 0.02 * pitch_angle_acc;
    #endif

    // Reset loop timer for the next iteration
    previous_time = current_time;

    // Read receiver inputs
    read_receiver();
  
    switch (steering_manner)
    {
      case 1:
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
        break;

      case 2:
        // Calculate desired rates based on receiver inputs
        DesiredAngleRoll = 0.15 * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.15 * (ReceiverValue[1] - 1500);
        InputThrottle = ReceiverValue[2];
        DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

        // Calculate errors for PID control
        ErrorAngleRoll = DesiredAngleRoll - roll_angle_gyro_fusion;
        ErrorAnglePitch = DesiredAnglePitch - pitch_angle_gyro_fusion;
        ErrorRateYaw = DesiredRateYaw - RateYaw;

        // Apply PID control for roll
        pid_equation(ErrorAngleRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
        InputRoll = PIDReturn[0];
        PrevErrorAngleRoll = PIDReturn[1];
        PrevItermAngleRoll = PIDReturn[2];

        // Apply PID control for pitch
        pid_equation(ErrorAnglePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
        InputPitch = PIDReturn[0];
        PrevErrorAnglePitch = PIDReturn[1];
        PrevItermAnglePitch = PIDReturn[2];

        // Apply PID control for yaw  !!! rate nog angle !!!!
        pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw = PIDReturn[0];
        PrevErrorRateYaw = PIDReturn[1];
        PrevItermRateYaw = PIDReturn[2];
        break;
    default:
      break;
    }
    

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

  LoopTimer = micros();
  }

  // print debug values to USB
  #ifdef debug_graph
    if (micros() - LoopTimer3 > 100000){
    Serial.print(RateRoll,0);

    Serial.print("\t");
    Serial.print(RatePitch,0);

    Serial.print("\t");
    Serial.print(RateYaw,0);

    Serial.print("\t");
    Serial.print(DesiredRateRoll,0);

    Serial.print("\t");
    Serial.print(DesiredRatePitch,0);

    Serial.print("\t");
    Serial.print(DesiredRateYaw,0);

    Serial.print("\t");
    Serial.print(InputThrottle,0);

    Serial.print("\t");
    Serial.print(ErrorRateRoll,0);

    Serial.print("\t");
    Serial.print(ErrorRatePitch,0);

    Serial.print("\t");
    Serial.print(ErrorRateYaw,0);

    Serial.print("\t");
    Serial.print(MotorInput4,0);

    Serial.print("\t");
    Serial.print(MotorInput3,0);

    Serial.print("\t");
    Serial.print(MotorInput2,0);

    Serial.print("\t");
    Serial.println(MotorInput1,0);
    LoopTimer3 = micros();
    }
  #endif  

  // print debug values to USB
  #ifdef debug_text
    if (micros() - LoopTimer3 > 100000){
    switch (steering_manner)
    {
      case 1:
        Serial.print("R_Roll:");
        //Serial.print("\t");
        Serial.print(RateRoll,0);
        Serial.print("\t");
        Serial.print("R_Pitch:");
        //Serial.print("\t");
        Serial.print(RatePitch,0);
        Serial.print("\t");
        Serial.print("R_Yaw:  ");
        //Serial.print("\t");
        Serial.print(RateYaw,0);
        Serial.print("\t");
        Serial.print("D_R_roll:");
        //Serial.print("\t");
        Serial.print(DesiredRateRoll,0);
        Serial.print("\t");
        Serial.print("Angle_Roll:");
        //Serial.print("\t");
        Serial.print(roll_angle_gyro_fusion,0);
        Serial.print("\t");
        Serial.print("Angle_Pitch:");
        //Serial.print("\t");
        Serial.print(pitch_angle_gyro_fusion,0);
        Serial.print("\t");
        Serial.print("D_R_pitch:");
        //Serial.print("\t");
        Serial.print(DesiredRatePitch,0);
        Serial.print("\t");
        Serial.print("D_R_yaw:");
        //Serial.print("\t");
        Serial.print(DesiredRateYaw,0);
        Serial.print("\t");
        Serial.print("D_power:");
        //Serial.print("\t");
        Serial.print(InputThrottle,0);
        Serial.print("\t");
        Serial.print("E_R_roll:");
        //Serial.print("\t");
        Serial.print(ErrorRateRoll,0);
        Serial.print("\t");
        Serial.print("E_R_pitch:  ");
        //Serial.print("\t");
        Serial.print(ErrorRatePitch,0);
        Serial.print("\t");
        Serial.print("E_R_yaw:");
        //Serial.print("\t");
        Serial.print(ErrorRateYaw,0);
        Serial.print("\t");
        Serial.print("M4:");
        //Serial.print("\t");
        Serial.print(MotorInput4,0);
        Serial.print("\t");
        Serial.print("M3:");
        //Serial.print("\t");
        Serial.print(MotorInput3,0);
        Serial.print("\t");
        Serial.print("M2:");
        //Serial.print("\t");
        Serial.print(MotorInput2,0);
        Serial.print("\t");
        Serial.print("M1:");
        //Serial.print("\t");
        Serial.println(MotorInput1,0);
        break;

      case 2:
        Serial.print("V:");
        //Serial.print("\t");
        Serial.print(Voltage,2);
        Serial.print("\t");
        Serial.print("I:");
        //Serial.print("\t");
        Serial.print(Current,2);
        Serial.print("\t");
        Serial.print("R_A_Roll:");
        //Serial.print("\t");
        Serial.print(roll_angle_gyro_fusion,0);
        Serial.print("\t");
        Serial.print("R_A_Pitch:");
        //Serial.print("\t");
        Serial.print(pitch_angle_gyro_fusion,0);
        Serial.print("\t");
        Serial.print("R_R_Yaw:");
        //Serial.print("\t");
        Serial.print(RateYaw,0);
        Serial.print("\t");
        Serial.print("D_A_roll:");
        //Serial.print("\t");
        Serial.print(DesiredAngleRoll,0);
        Serial.print("\t");
        Serial.print("D_A_pitch:");
        //Serial.print("\t");
        Serial.print(DesiredAnglePitch,0);
        Serial.print("\t");
        Serial.print("D_R_yaw:");
        //Serial.print("\t");
        Serial.print(DesiredRateYaw,0);
        Serial.print("\t");
        Serial.print("D_power:");
        //Serial.print("\t");
        Serial.print(InputThrottle,0);
        Serial.print("\t");
        Serial.print("E_A_roll:");
        //Serial.print("\t");
        Serial.print(ErrorAngleRoll,0);
        Serial.print("\t");
        Serial.print("E_A_pitch:  ");
        //Serial.print("\t");
        Serial.print(ErrorAnglePitch,0);
        Serial.print("\t");
        Serial.print("E_R_yaw:");
        //Serial.print("\t");
        Serial.print(ErrorRateYaw,0);
        Serial.print("\t");
        Serial.print("I_Roll:");
        //Serial.print("\t");
        Serial.print(PrevItermAngleRoll,0);
        Serial.print("\t");
        Serial.print("In_Roll:");
        //Serial.print("\t");
        Serial.print(InputRoll,0);
        Serial.print("\t");
        Serial.print("In_Pitch:");
        //Serial.print("\t");
        Serial.print(InputPitch,0);
        Serial.print("\t");
        Serial.print("In_Yaw:  ");
        //Serial.print("\t");
        Serial.print(InputYaw,0);
        Serial.print("\t");
        Serial.print("M4:");
        //Serial.print("\t");
        Serial.print(MotorInput4,0);
        Serial.print("\t");
        Serial.print("M3:");
        //Serial.print("\t");
        Serial.print(MotorInput3,0);
        Serial.print("\t");
        Serial.print("M2:");
        //Serial.print("\t");
        Serial.print(MotorInput2,0);
        Serial.print("\t");
        Serial.print("M1:");
        //Serial.print("\t");
        Serial.println(MotorInput1,0);
        break;
    default:
      break;
    }  
    
    LoopTimer3 = micros();
    }
  #endif  

  // builtin LED flashing when in loopmodus
  if (micros() - LoopTimer2 > 400000) {
  // Toggle LED state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Reset LoopTimer for the next iteration
  LoopTimer2 = micros();
  }
}
