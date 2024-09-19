#include <Arduino.h>

#define IBUS_READ  // switch between IBUS (130Hz/7.7ms) and PPM (50Hz/20ms) reading from Reciever. Think about changing pins.

// Include necessary libraries
//#include <Wire.h>
#ifndef IBUS_READ
  #include <PulsePosition.h>
#endif
//#include "wiring.h"
#include "GyroSignals.h"
#include "Barometer.h"
#include <TinyGPS++.h>
#include "IBusReceiver.h"

//#define GPS_sensor_active
#define pressure_sensor_active

#define debug
//#define debug_plot_graph // format "var:value/t"
#define debug_teleplot_graph  // format ">var:value/n"
#define debug_barometer
#define debug_receiver
//#define debug_GPS



#define sensor_fusion

// teensy 4.1 pinconfiguration
// I2C wire = pins 19 for SCL and 18 for SDA. (I2C wire1 = pins 16 for SCL1 and 17 for SDA1. I2C wire2 = pins 24 for SCL2 and 25 for SDA2)
// GPS Serial2 = pins 7 for RX2 and 8 for TX2. Attention TX to RX and RX to TX.
#define RecieverPin 14 //PPM signal reciever
#define LedGreenPin 6 //LED Green Voltage battery
#define LedRedPin 5 //LED Red energy battery to low
#define Motor1Pin 1 //CCW Front Right
#define Motor2Pin 2 //CW Back Right
#define Motor3Pin 3 //CCW Back Left
#define Motor4Pin 4 //CW Front Left
#define VoltageBatteryPin 15 //Battery Voltage measuring 
#define CurrentBatteryPin 21 //Battery Current measuring

#define steering_manner 2 // 1 for rate ; 2 for angle

// Global variables for gyroscope readings
float RatePitch, RateRoll, RateYaw;
float CalibrationGyroPitch, CalibratioGyroRoll, CalibrationGyroYaw;
int CalibrationNumber = 2000;
float roll_angle_gyro, pitch_angle_gyro, yaw_angle_gyro;
float roll_angle_gyro_fusion, pitch_angle_gyro_fusion;
float roll_angle_acc , pitch_angle_acc;
float velocityX , velocityY , velocityZ ;
float positionX , positionY , positionZ ; 
int switchB_State;
int switchC_State;

// Global variables for acceleration readings
float AccX,AccY,AccZ;
float CalibrationAccX,CalibrationAccY,CalibrationAccZ;

// Global variables for Barometer readings
float pressure;
float cTemp;
float initialPressure;
float relativeAltitude;

// Create an IBusReceiver object for Serial7 (change Serial7 to Serial1 or appropriate Serial if needed)
IBusReceiver ibus(Serial7);

// Global variables for RC receiver inputs
#ifndef IBUS_READ
PulsePositionInput ReceiverInput(RISING);
#endif
int ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0,0,0};
int ChannelNumber = 0;

// Global variables for battery status
float Voltage, Current, BatteryEnergyPercRemaining, BatteryEnergyAtStart;
float BatteryEnergyConsumed = 0.0;
float BatteryEnergyDefault4S = 3700.0;  // Capacity of 4S battery in mAh
float BatteryEnergyDefault6S = 4000.0;  // Capacity of 6S battery in mAh
float BatteryEnergyDefault; // To store the selected battery capacity
unsigned long lastUpdateTime = 0;
const float TimeStep = 0.004; // Update period in seconds (4 ms)
int batteryType = 0; // 0 for unknown, 4 for 4S, 6 for 6S

// Timer variable for main loop
uint32_t LoopTimer;
uint32_t LoopTimer2;
uint32_t LoopTimer3;
uint32_t LoopTimer4;
uint32_t LoopTimer5;
uint32_t LoopTimer6;
uint32_t current_time;
float time_difference;
uint32_t previous_time;


// PID constants for roll, pitch, and yaw control
float PRateRoll,PRateRoll_tst;
float PRatePitch,PRatePitch_tst;
float PRateYaw ;
float IRateRoll,IRateRoll_tst;
float IRatePitch;
float IRateYaw;
float DRateRoll,DRateRoll_tst;
float DRatePitch,DRatePitch_tst;
float DRateYaw;
float PAngleRoll,PAngleRoll_tst;
float PAnglePitch,PAnglePitch_tst;
float PAngleYaw ;
float IAngleRoll,IAngleRoll_tst;
float IAnglePitch,IAnglePitch_tst;
float IAngleYaw;
float DAngleRoll,DAngleRoll_tst;
float DAnglePitch,DAnglePitch_tst;
float DAngleYaw;

// Array to store PID output and related variables
float PIDReturn[] = {0, 0, 0, 0, 0};

// Variables for PID reset
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevPtermRateRoll, PrevPtermRatePitch, PrevPtermRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PrevDtermRateRoll, PrevDtermRatePitch, PrevDtermRateYaw;
float PrevErrorAngleRoll, PrevErrorAnglePitch, PrevErrorAngleYaw;
float PrevPtermAngleRoll, PrevPtermAnglePitch, PrevPtermAngleYaw;
float PrevItermAngleRoll, PrevItermAnglePitch, PrevItermAngleYaw;
float PrevDtermAngleRoll, PrevDtermAnglePitch, PrevDtermAngleYaw;

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

// Declare Class Barometer
Barometer barometer(0x76);  // BMP280 I2C address

// Declare GPS sensor
TinyGPSPlus gps;
const uint32_t GPSBaud = 38400;  // GY_GPSV3_NEO_M9N GPSBaud = 38400 : M10A-5883 GPSBaud = 9600
char data;

// Function to read battery voltage and current
void battery_voltage(void) 
{
  // Read voltage and current from analog pins
  Voltage = (float)analogRead(VoltageBatteryPin) / 40.3;
  Current = (float)analogRead(CurrentBatteryPin) / 2.0 - 0.25;  //0.089 scale current to amps 1/75 for Mateksys FCHUB-12S fullrange 440A max 3,3V.
}

// Function to detect battery type based on voltage
void detect_battery_type(float voltage)
{
  if (voltage > 18.0) {  // If voltage > 18V, assume it's a 6S battery
    batteryType = 6;
    BatteryEnergyDefault = BatteryEnergyDefault6S;  // Set capacity for 6S battery
  } 
  else if (voltage > 12.0 && voltage <= 18.0) {  // If voltage between 12V and 18V, assume 4S
    batteryType = 4;
    BatteryEnergyDefault = BatteryEnergyDefault4S;  // Set capacity for 4S battery
  }
}

// Function to estimate remaining capacity based on voltage (idle/no load)
float estimate_capacity_from_voltage(float voltage, int batteryType)
{
  // 4S voltage-to-capacity approximation
  if (batteryType == 4) {
    if (voltage >= 16.8) return 100.0;  // Fully charged (4.2V per cell)
    if (voltage >= 15.6) return 90.0;
    if (voltage >= 15.2) return 80.0;
    if (voltage >= 14.8) return 70.0;
    if (voltage >= 14.4) return 60.0;
    if (voltage >= 14.0) return 50.0;
    if (voltage >= 13.6) return 40.0;
    if (voltage >= 13.2) return 30.0;
    if (voltage >= 12.8) return 20.0;
    if (voltage >= 12.0) return 10.0;
    return 0.0;
  }
  
  // 6S voltage-to-capacity approximation
  else if (batteryType == 6) {
    if (voltage >= 25.2) return 100.0;  // Fully charged (4.2V per cell)
    if (voltage >= 23.4) return 90.0;
    if (voltage >= 22.8) return 80.0;
    if (voltage >= 22.2) return 70.0;
    if (voltage >= 21.6) return 60.0;
    if (voltage >= 21.0) return 50.0;
    if (voltage >= 20.4) return 40.0;
    if (voltage >= 19.8) return 30.0;
    if (voltage >= 19.2) return 20.0;
    if (voltage >= 18.0) return 10.0;
    return 0.0;
  }
  
  return 0.0;  // Default, if unknown
}

#ifndef IBUS_READ
// Function to read PPM signals from RC receiver via digitalinputpin.
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
#endif

#ifdef IBUS_READ
// Function to read IBUS signals from RC receiver via RX UART pin.
void read_receiver(void) {
  if (ibus.readChannels()) {
      for (int i = 1; i <= IBUS_CHANNELS; i++) {
        ReceiverValue[i-1] = int(ibus.getChannelValue(i));
      }
  }
}  
#endif

bool checkGPSConnection() {
  // Check for data from GPS module
  for (int i = 0; i < 10; ++i) {
    if (Serial2.available()) {
      return true;
    }
    delay(500); // Wait 500ms before checking again
  }
  return false;
}

// Function to clamp values within a specified range
float clamp(float value, float min_val, float max_val) {
  if (value > max_val) return max_val;
  if (value < min_val) return min_val;
  return value;
}

// Function to calculate PID output
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm,float deltaTime = 0.004) {
  // Calculate proportional, integral, and derivative terms
  // Calculate proportional term
  float Pterm = P * Error;

  // Calculate integral term (trapezoidal rule for integration)
  float Iterm = PrevIterm + I * (Error + PrevError) * deltaTime  / 2;

  // Clamp the integral term to prevent windup
  Iterm = clamp(Iterm, -400, 400);

    // Calculate derivative term (protect against division by zero in case of small deltaTime)
  float Dterm = 0;
  if (deltaTime > 0) {
    Dterm = D * (Error - PrevError) / deltaTime;
  }
  
  float PIDOutput = Pterm + Iterm + Dterm;
  
  // Clamp PID output to prevent excessive output
  PIDOutput = clamp(PIDOutput, -400, 400);

  // Store PID output and related variables
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
  PIDReturn[3] = Pterm;
  PIDReturn[4] = Dterm;
}

int determine3SwitchState(int receiverValue) {
    if (receiverValue >= 900 && receiverValue <= 1100) {
        return 1;  // First state for values around 1000
    } else if (receiverValue >= 1400 && receiverValue <= 1600) {
        return 2;  // Second state for values around 1500
    } else if (receiverValue >= 1900 && receiverValue <= 2100) {
        return 3;  // Third state for values around 2000
    } else {
        return 0;  // Default state if the value is outside all ranges
    }
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
  
  // delay to let the sensors first settle after powerup
  delay(1000);

  #ifdef GPS_sensor_active
  // serial2 setup for GPS sensor
  Serial2.begin(GPSBaud);
  #endif

  // Initialize iBus communication at 115200 baud rate
  ibus.begin(115200);  

  #ifdef debug
    Serial.println("Begin initializing I2C"); 
  #endif

  // Initialize I2C communication and sensors
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  #ifdef debug
    Serial.println("Begin initializing Gyro/Acc sensor MPU6050"); 
  #endif
  
  // Initialize Gyro and Acceleration sensor
  gyroSignals.init();

  #ifdef debug 
    Serial.println("Init MPU6050 successfull");
    Serial.println("Start calibration Gyro and Acceleration sensor");
  #endif
    
  // Calibrate gyroscope readings
  gyroSignals.calibrate(CalibratioGyroRoll, CalibrationGyroPitch, CalibrationGyroYaw,
                             CalibrationAccX, CalibrationAccY, CalibrationAccZ, CalibrationNumber);

  #ifdef debug 
    Serial.println();
    Serial.println("Calibration successfull");
    Serial.print("Calibrationsamples");
    Serial.print("\t");
    Serial.println(CalibrationNumber);
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

  #ifdef pressure_sensor_active
    #ifdef debug
      Serial.println("Begin initializing Barometersensor BMP280"); 
    #endif

    // Initialize barometer sensor BMP280
    barometer.begin();

    #ifdef debug 
      Serial.println("Init BMP280 successfull");
    #endif
    
    // Initial pressure for Altitude reference
    initialPressure = barometer.getInitialPressure();

    #ifdef debug 
      Serial.println("Initial Pressure");
      Serial.print(" ");
      Serial.print(initialPressure,4);
      Serial.println(" mbar");
      Serial.println("");
    #endif
  #endif

  #ifdef GPS_sensor_active
    // Check GPS connection
    if (!checkGPSConnection()) {
      Serial.println("GPS module not detected. Please check the connection.");
      while (1); // halt the program
    } else {
      Serial.println("GPS module connected successfully.");
      Serial.println("");
    }
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

  // Initialize the starting battery energy based on voltage
  battery_voltage();  // Initial voltage reading
  BatteryEnergyAtStart = BatteryEnergyDefault * estimate_capacity_from_voltage(Voltage, batteryType) / 100.0;
  lastUpdateTime = millis();

  #ifndef IBUS_READ
  // Initialize RC receiver
  ReceiverInput.begin(RecieverPin);
  #endif
  
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
    PAngleRoll = 1.3; // 1.3 :resonantie op 10
    PAnglePitch = 1.3; //resonantie op 10
    PAngleYaw = 2;
    IAngleRoll = 1.1; // 3.3
    IAnglePitch = IAngleRoll;
    IAngleYaw = 0;
    DAngleRoll = 0.2; //0.28
    DAnglePitch = DAngleRoll;
    DAngleYaw = 0.0;
    break;

  default:
    break;
  }

  // Start loop timer
  LoopTimer = micros();
  LoopTimer2 = micros();
  LoopTimer3 = micros();
  LoopTimer4 = micros();
  LoopTimer5 = micros();
  LoopTimer6 = micros();
  previous_time = micros();
}


void loop()
    {    // sensor read every 4ms
    if (micros() - LoopTimer4 > 4000){
        //Serial.print(micros() - LoopTimer4);
        //Serial.print(" ");
        LoopTimer4 = micros();

        // Read gyroscope data and correct with calibrationfactors
        gyroSignals.readGyroData(RateRoll, RatePitch, RateYaw);
        RateRoll -= CalibratioGyroRoll;  
        RatePitch -= CalibrationGyroPitch; 
        RateYaw -= CalibrationGyroYaw; 
        //Serial.println(RateYaw);

        // Read acceleration data and correct with calibrationfactors
        gyroSignals.readAccelData(AccX, AccY, AccZ);
        AccX -= CalibrationAccX;
        AccY -= CalibrationAccY;
        AccZ = AccZ+(1-CalibrationAccZ);

        current_time = micros();
        time_difference = (current_time - previous_time)/1000000.0; // scale microseconds to seconds
        // Reset loop timer for the next iteration
        previous_time = current_time;

        // sensor calculations
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

        // sensor fusion
        // sensor fusion with complementary filter gyro and accelerator sensor
        #ifdef sensor_fusion 
          roll_angle_gyro_fusion = 0.98 * roll_angle_gyro_fusion + 0.02 * roll_angle_acc;
          pitch_angle_gyro_fusion = 0.98 * pitch_angle_gyro_fusion + 0.02 * pitch_angle_acc;
        #endif
        
        // Convert roll, pitch, yaw to radians for rotation matrix
        float roll_rad = roll_angle_gyro_fusion * PI / 180.0;
        float pitch_rad = pitch_angle_gyro_fusion* PI / 180.0;
        float yaw_rad = yaw_angle_gyro * PI / 180.0;


        // Rotation matrix to convert from body frame to world frame
        float R11 = cos(pitch_rad) * cos(yaw_rad);
        float R12 = cos(pitch_rad) * sin(yaw_rad);
        float R13 = -sin(pitch_rad);
        float R21 = sin(roll_rad) * sin(pitch_rad) * cos(yaw_rad) - cos(roll_rad) * sin(yaw_rad);
        float R22 = sin(roll_rad) * sin(pitch_rad) * sin(yaw_rad) + cos(roll_rad) * cos(yaw_rad);
        float R23 = sin(roll_rad) * cos(pitch_rad);
        float R31 = cos(roll_rad) * sin(pitch_rad) * cos(yaw_rad) + sin(roll_rad) * sin(yaw_rad);
        float R32 = cos(roll_rad) * sin(pitch_rad) * sin(yaw_rad) - sin(roll_rad) * cos(yaw_rad);
        float R33 = cos(roll_rad) * cos(pitch_rad);

        // Calculate world-frame acceleration
        float AccX_world = R11 * AccX + R12 * AccY + R13 * AccZ;
        float AccY_world = R21 * AccX + R22 * AccY + R23 * AccZ;
        float AccZ_world = R31 * AccX + R32 * AccY + R33 * AccZ - 9.81; // Subtract gravity

        // Integrate acceleration to get velocity
        velocityX += AccX_world * time_difference;
        velocityY += AccY_world * time_difference;
        velocityZ += AccZ_world * time_difference;

        // Integrate velocity to get position
        positionX += velocityX * time_difference;
        positionY += velocityY * time_difference;
        positionZ += velocityZ * time_difference;
    }
    // Barometer sensor read every 100ms
    if (micros() - LoopTimer5 > 100000) {
        LoopTimer5 = micros();
        BaroData data = barometer.readData();
        pressure = data.pressure;
        cTemp = data.cTemp;
        relativeAltitude = barometer.calculateAltitude(pressure,cTemp) - barometer.calculateAltitude(initialPressure,cTemp);
    }

    // read reciever data
    #ifdef IBUS_READ
    // Check for new IBUS data every 7700us
    if (micros() - LoopTimer6 > 7700) {
        LoopTimer6 = micros();
        read_receiver();
    }
    #endif

    #ifndef IBUS_READ
    // Check for new PPM data every 20000us
    if (micros() - LoopTimer6 > 20000) {
        LoopTimer6 = micros();
        read_receiver();
    }
    #endif

    // sensorfusion and PID calc every 4ms
    if (micros() - LoopTimer > 4000){
      LoopTimer = micros();
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
          PrevPtermRateRoll = PIDReturn[3];
          PrevDtermRateRoll = PIDReturn[4];

          // Apply PID control for pitch
          pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
          InputPitch = PIDReturn[0];
          PrevErrorRatePitch = PIDReturn[1];
          PrevItermRatePitch = PIDReturn[2];
          PrevPtermRatePitch = PIDReturn[3];
          PrevDtermRatePitch = PIDReturn[4];

          // Apply PID control for yaw
          pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
          InputYaw = PIDReturn[0];
          PrevErrorRateYaw = PIDReturn[1];
          PrevItermRateYaw = PIDReturn[2];
          PrevPtermRateYaw = PIDReturn[3];
          PrevDtermRateYaw = PIDReturn[4];
          break;

        case 2:
          switchB_State = determine3SwitchState(ReceiverValue[8]);
          switchC_State = determine3SwitchState(ReceiverValue[9]);
          // Calculate desired rates based on receiver inputs
          DesiredAngleRoll = 0.07 * (ReceiverValue[0] - 1500);
          DesiredAnglePitch = 0.07 * (ReceiverValue[1] - 1500);
          InputThrottle = ReceiverValue[2];
          DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
          switch (switchB_State)
          {
          case 1:
            PAngleRoll_tst = PAngleRoll;
            IAngleRoll_tst = IAngleRoll;
            DAngleRoll_tst = DAngleRoll;
            PAnglePitch_tst = PAnglePitch;
            IAnglePitch_tst = IAngleRoll;
            DAnglePitch_tst = DAngleRoll;
            break;
          case 2:        
            switch (switchC_State)
            {
            case 1:
                // Smoothly update P gain
                PAngleRoll_tst = 0.9 * PAngleRoll_tst + 0.1 * (0.0025 * (ReceiverValue[4]- 1000));
                // Ensure IAngleRoll_tst does not go below 0
                PAngleRoll_tst = max(PAngleRoll_tst, 0.0);
                PAnglePitch_tst = PAngleRoll_tst;
              break;
            case 2:
                //Smoothly update I gain
                IAngleRoll_tst = 0.9 * IAngleRoll_tst + 0.1 * (0.01 * (ReceiverValue[4] - 1000));
                // Ensure IAngleRoll_tst does not go below 0
                IAngleRoll_tst = max(IAngleRoll_tst, 0.0);
                IAnglePitch_tst = IAngleRoll_tst;
              break;
            case 3:
                //Smoothly update D gain;
                DAngleRoll_tst = 0.9 * DAngleRoll_tst + 0.1 * (0.001 * (ReceiverValue[4] - 1005));
                // Ensure DAngleRoll_tst does not go below 0
                DAngleRoll_tst = max(DAngleRoll_tst, 0.0);
                DAnglePitch_tst = DAngleRoll_tst;
              break;  
            default:
              break;
            }
            break;
          default:
            break;
          }
          // Calculate errors for PID control
          ErrorAngleRoll = DesiredAngleRoll - roll_angle_gyro_fusion;
          ErrorAnglePitch = DesiredAnglePitch - pitch_angle_gyro_fusion;
          ErrorRateYaw = DesiredRateYaw - RateYaw;

          // Apply PID control for roll
          pid_equation(ErrorAngleRoll, PAngleRoll_tst, IAngleRoll_tst, DAngleRoll_tst, PrevErrorAngleRoll, PrevItermAngleRoll);
          InputRoll = PIDReturn[0];
          PrevErrorAngleRoll = PIDReturn[1];
          PrevItermAngleRoll = PIDReturn[2];
          PrevPtermAngleRoll = PIDReturn[3];
          PrevDtermAngleRoll = PIDReturn[4];

          // Apply PID control for pitch
          pid_equation(ErrorAnglePitch, PAnglePitch_tst, IAnglePitch_tst, DAnglePitch_tst, PrevErrorAnglePitch, PrevItermAnglePitch);
          InputPitch = PIDReturn[0];
          PrevErrorAnglePitch = PIDReturn[1];
          PrevItermAnglePitch = PIDReturn[2];
          PrevPtermAnglePitch = PIDReturn[3];
          PrevDtermAnglePitch = PIDReturn[4];

          // Apply PID control for yaw  !!! rate not angle !!!!
          pid_equation(ErrorRateYaw, PAngleYaw, IAngleYaw, DAngleYaw, PrevErrorAngleYaw, PrevItermAngleYaw);
          InputYaw = PIDReturn[0];
          PrevErrorAngleYaw = PIDReturn[1];
          PrevItermAngleYaw = PIDReturn[2];
          PrevPtermAngleYaw = PIDReturn[3];
          PrevDtermAngleYaw = PIDReturn[4];
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

    // Update battery status every time step
    unsigned long currentTime = millis();
    if ((currentTime - lastUpdateTime) >= TimeStep * 1000) 
    {
      lastUpdateTime = currentTime;

      // Read voltage and current
      battery_voltage();

      // Integrate current to update energy consumption (Coulomb Counting)
      BatteryEnergyConsumed += Current * 1000.0 * TimeStep / 3600.0; // mAh used

      // Calculate percentage of remaining energy
      BatteryEnergyPercRemaining = (BatteryEnergyAtStart - BatteryEnergyConsumed) / BatteryEnergyDefault * 100.0;

      // If battery is idle, update the SoC based on voltage (idle reading)
      if (abs(Current) < 0.3)  // If the current is very small, consider battery at idle
      {
        BatteryEnergyPercRemaining = estimate_capacity_from_voltage(Voltage, batteryType);
      }

      // Control LED based on battery level
      if (BatteryEnergyPercRemaining <= 30.0) digitalWrite(LedRedPin, HIGH);
      else digitalWrite(LedRedPin, LOW);
    }
    }

  // print debug values to USB every 100ms
  #ifdef debug
    if (micros() - LoopTimer3 > 100000){
    LoopTimer3 = micros();
    #ifdef debug_teleplot_graph
      #ifdef debug_receiver
        for (int i = 1; i <= IBUS_CHANNELS; i++) 
          {
          Serial.print(">Ch");
          Serial.print(i);
          Serial.print(":");
          Serial.print(ReceiverValue[i-1]);
          Serial.print("\n");
          }
      #endif
      #ifdef debug_barometer
        Serial.print(">Temp:");
        Serial.print(cTemp);
        Serial.print("\n");
        Serial.print(">Press:");
        Serial.print(pressure, 3);
        Serial.print("\n");
        Serial.print(">Alt:");
        Serial.println(relativeAltitude, 0);
      #endif
      #ifdef debug_GPS
        while (Serial2.available() > 0) {
        data = Serial2.read();
        if (gps.encode(data)) {
          if (gps.location.isValid()) {
            Serial.print(">Lat:");
            Serial.println(gps.location.lat(), 6);
            Serial.print(">Long:");
            Serial.println(gps.location.lng(), 6);
          } else {
            Serial.println("Location: Not Available");
          }
          if (gps.altitude.isValid()) {
            Serial.print(">Alt_gps:");
            Serial.println(gps.altitude.meters());
          } else {
            Serial.println("Altitude: Not Available");
          }
          if (gps.speed.isValid()) {
            Serial.print(">V_gps:");
            Serial.println(gps.speed.kmph());
          } else {
            Serial.println("Speed: Not Available");
          }
          if (gps.satellites.isValid()) {
            Serial.print(">Sat_nr_gps:");
            Serial.println(gps.satellites.value());
          } else {
            Serial.println("Number of Satellites: Not Available");
          }
          if (gps.course.isValid()) {
            Serial.print(">Course_(heading)_gps:");
            Serial.println(gps.course.deg());
          } else {
            Serial.println("Course (heading): Not Available");
          }
          if (gps.hdop.isValid()) {
            Serial.print(">HDOP:");//horizontal dilution of precision 
            Serial.println(gps.hdop.value());
          } else {
            Serial.println("hdop: Not Available");
          }
          if (gps.date.isValid()) {
            Serial.print("Date_UTC:");
            Serial.print(gps.date.month());
            Serial.print("/");
            Serial.print(gps.date.day());
            Serial.print("/");
            Serial.println(gps.date.year());
          } else {
            Serial.println("Date: Not Available");
          }
          if (gps.time.isValid()) {
            Serial.print("Time_UTC:");
            if (gps.time.hour() < 10) Serial.print("0");
            Serial.print(gps.time.hour());
            Serial.print(":");
            if (gps.time.minute() < 10) Serial.print("0");
            Serial.print(gps.time.minute());
            Serial.print(":");
            if (gps.time.second() < 10) Serial.print("0");
            Serial.println(gps.time.second());
            Serial.print(">Time_UTC:");
            Serial.println(gps.time.value());
          } else {
            Serial.println("Time: Not Available");
          }
        }
        }
      #endif
    switch (steering_manner)
    {
      case 1:
        Serial.print(">V:");
        Serial.println(Voltage,2);
        Serial.print(">I:");
        Serial.println(Current,2);
        Serial.print(">R_A_roll:");
        Serial.println(roll_angle_gyro_fusion,0);
        Serial.print(">R_A_pitch:");
        Serial.println(pitch_angle_gyro_fusion,0);
        Serial.print(">R_A_yaw:");
        Serial.println(yaw_angle_gyro,0);
        Serial.print(">R_R_roll:");
        Serial.println(RateRoll,0);
        Serial.print(">R_R_pitch:");
        Serial.println(RatePitch,0);
        Serial.print(">R_R_yaw:");
        Serial.println(RateYaw,0);
        Serial.print(">D_A_roll:");
        Serial.println(DesiredAngleRoll,0);
        Serial.print(">D_A_pitch:");
        Serial.println(DesiredAnglePitch,0);
        Serial.print(">D_R_yaw:");
        Serial.println(DesiredRateYaw,0);
        Serial.print(">D_R_roll:");
        Serial.println(DesiredRateRoll,0);
        Serial.print(">D_R_pitch:");
        Serial.println(DesiredRatePitch,0);
        Serial.print(">D_R_yaw:");
        Serial.println(DesiredRateYaw,0);
        Serial.print(">D_power:");
        Serial.println(InputThrottle,0);
        Serial.print(">E_A_roll:");
        Serial.println(ErrorAngleRoll,0);
        Serial.print(">E_A_pitch:  ");
        Serial.println(ErrorAnglePitch,0);
        Serial.print(">E_R_yaw:");
        Serial.println(ErrorRateYaw,0);
        Serial.print(">P_roll:");
        Serial.println(PrevPtermAngleRoll,0);
        Serial.print(">I_roll:");
        Serial.println(PrevItermAngleRoll,0);
        Serial.print(">D_roll:");
        Serial.println(PrevDtermAngleRoll,0);
        Serial.print(">In_roll:");
        Serial.println(InputRoll,0);
        Serial.print(">In_pitch:");
        Serial.println(InputPitch,0);
        Serial.print(">In_yaw:  ");
        Serial.println(InputYaw,0);
        Serial.print(">M4:");
        Serial.println(MotorInput4,0);
        Serial.print(">M3:");
        Serial.println(MotorInput3,0);
        Serial.print(">M2:");
        Serial.println(MotorInput2,0);
        Serial.print(">M1:");
        Serial.println(MotorInput1,0);
        Serial.print(">PrateR:");
        Serial.println(PAngleRoll_tst,1);
        Serial.print(">IrateR:");
        Serial.println(IAngleRoll_tst,1);
        Serial.print(">DrateR:");
        Serial.println(DAngleRoll_tst,2); 
        Serial.print(">SWB:");
        Serial.println(switchB_State);
        Serial.print(">SWC:");
        Serial.println(switchC_State); 
        Serial.print(">BattRemain:");
        Serial.println(BatteryEnergyPercRemaining,0);
        Serial.print(">VelocX:");
        Serial.println(velocityX,2);
        Serial.print(">PosX:");
        Serial.println(positionX,2);
        break;
      case 2:
        Serial.print(">V:");
        Serial.println(Voltage,2);
        Serial.print(">I:");
        Serial.println(Current,2);
        Serial.print(">R_A_roll:");
        Serial.println(roll_angle_gyro_fusion,0);
        Serial.print(">R_A_pitch:");
        Serial.println(pitch_angle_gyro_fusion,0);
        Serial.print(">R_A_yaw:");
        Serial.println(yaw_angle_gyro,0);
        Serial.print(">R_R_roll:");
        Serial.println(RateRoll,0);
        Serial.print(">R_R_pitch:");
        Serial.println(RatePitch,0);
        Serial.print(">R_R_yaw:");
        Serial.println(RateYaw,0);
        Serial.print(">D_A_roll:");
        Serial.println(DesiredAngleRoll,0);
        Serial.print(">D_A_pitch:");
        Serial.println(DesiredAnglePitch,0);
        Serial.print(">D_R_yaw:");
        Serial.println(DesiredRateYaw,0);
        Serial.print(">D_R_roll:");
        Serial.println(DesiredRateRoll,0);
        Serial.print(">D_R_pitch:");
        Serial.println(DesiredRatePitch,0);
        Serial.print(">D_R_yaw:");        
        Serial.println(DesiredRateYaw,0);
        Serial.print(">D_power:");
        Serial.println(InputThrottle,0);
        Serial.print(">E_A_roll:");
        Serial.println(ErrorAngleRoll,0);
        Serial.print(">E_A_pitch:  ");
        Serial.println(ErrorAnglePitch,0);
        Serial.print(">E_R_yaw:");
        Serial.println(ErrorRateYaw,0);
        Serial.print(">P_roll:");
        Serial.println(PrevPtermAngleRoll,0);
        Serial.print(">I_roll:");
        Serial.println(PrevItermAngleRoll,0);
        Serial.print(">D_roll:");
        Serial.println(PrevDtermAngleRoll,0);
        Serial.print(">In_roll:");
        Serial.println(InputRoll,0);
        Serial.print(">P_pitch:");
        Serial.println(PrevPtermAnglePitch,0);
        Serial.print(">I_pitch:");
        Serial.println(PrevItermAnglePitch,0);
        Serial.print(">D_pitch:");
        Serial.println(PrevDtermAnglePitch,0);
        Serial.print(">In_pitch:");
        Serial.println(InputPitch,0);
        Serial.print(">P_yaw:");
        Serial.println(PrevPtermAngleYaw,0);
        Serial.print(">I_yaw:");
        Serial.println(PrevItermAngleYaw,0);
        Serial.print(">D_yaw:");
        Serial.println(PrevDtermAngleYaw,0);
        Serial.print(">In_yaw:  ");
        Serial.println(InputYaw,0);
        Serial.print(">M4:");
        Serial.println(MotorInput4,0);
        Serial.print(">M3:");
        Serial.println(MotorInput3,0);
        Serial.print(">M2:");
        Serial.println(MotorInput2,0);
        Serial.print(">M1:");
        Serial.println(MotorInput1,0);
        Serial.print(">PrateR:");
        Serial.println(PAngleRoll_tst,1);
        Serial.print(">IrateR:");
        Serial.println(IAngleRoll_tst,1);
        Serial.print(">DrateR:");
        Serial.println(DAngleRoll_tst,2); 
        Serial.print(">SWB:");
        Serial.println(switchB_State);
        Serial.print(">SWC:");
        Serial.println(switchC_State);
        Serial.print(">BattStart:");
        Serial.println(BatteryEnergyAtStart,0); 
        Serial.print(">BattConsumed:");
        Serial.println(BatteryEnergyConsumed,2);
        Serial.print(">BattRemain:");
        Serial.println(BatteryEnergyPercRemaining,2);
        Serial.print(">VelocX:");
        Serial.println(velocityX,2);
        Serial.print(">PosX:");
        Serial.println(positionX,2);
        break;
    default:
      break;
    }  
    #endif 

    #ifdef debug_plot_graph
      #ifdef debug_receiver
      for (int i = 1; i <= IBUS_CHANNELS; i++) {
              Serial.print("Ch");
              Serial.print(i);
              Serial.print(": ");
              Serial.print(ReceiverValue[i-1]);
              Serial.print(" ");
      }
      Serial.println();
      #endif
      #ifdef debug_barometer
        Serial.print("Temperature: ");
        Serial.print(cTemp);
        Serial.print(" C\tPressure: ");
        Serial.print(pressure, 3);
        Serial.print(" mbar\trelativeAltitude: ");
        Serial.print(relativeAltitude, 0);
        Serial.println(" cm");
      #endif 
      #ifdef debug_GPS
        while (Serial2.available() > 0) {
        data = Serial2.read();
        if (gps.encode(data)) {
          if (gps.location.isValid()) {
            Serial.print("Latitude : ");
            Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
          } else {
            Serial.println("Location: Not Available");
          }
          if (gps.altitude.isValid()) {
            Serial.print("Altitude: ");
            Serial.print(gps.altitude.meters());
            Serial.println(" meters");
          } else {
            Serial.println("Altitude: Not Available");
          }
          if (gps.speed.isValid()) {
            Serial.print("Speed: ");
            Serial.print(gps.speed.kmph());
            Serial.println(" km/h");
          } else {
            Serial.println("Speed: Not Available");
          }
          if (gps.satellites.isValid()) {
            Serial.print("Number of Satellites: ");
            Serial.println(gps.satellites.value());
          } else {
            Serial.println("Number of Satellites: Not Available");
          }
          if (gps.course.isValid()) {
            Serial.print("Course (heading): ");
            Serial.print(gps.course.deg());
            Serial.println(" degrees");
          } else {
            Serial.println("Course (heading): Not Available");
          }
          if (gps.hdop.isValid()) {
            Serial.print("HDOP horizontal dilution of precision : ");
            Serial.print(gps.hdop.value());
            Serial.println(" ");
          } else {
            Serial.println("hdop: Not Available");
          }
          if (gps.date.isValid()) {
            Serial.print("Date UTC: ");
            Serial.print(gps.date.month());
            Serial.print("/");
            Serial.print(gps.date.day());
            Serial.print("/");
            Serial.println(gps.date.year());
          } else {
            Serial.println("Date: Not Available");
          }
          if (gps.time.isValid()) {
            Serial.print("Time UTC: ");
            if (gps.time.hour() < 10) Serial.print("0");
            Serial.print(gps.time.hour());
            Serial.print(":");
            if (gps.time.minute() < 10) Serial.print("0");
            Serial.print(gps.time.minute());
            Serial.print(":");
            if (gps.time.second() < 10) Serial.print("0");
            Serial.println(gps.time.second());
          } else {
            Serial.println("Time: Not Available");
          }
          Serial.println();
        }
        }
      #endif      
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
          Serial.print(MotorInput1,0);
          Serial.print("\t");
          Serial.print("PrateR:");
          //Serial.print("\t");
          Serial.print(PRateRoll_tst,1);
          Serial.print("\t");
          Serial.print("IrateR:");
          //Serial.print("\t");
          Serial.print(IRateRoll_tst,1);
          Serial.print("\t");
          Serial.print("DrateR:");
          //Serial.print("\t");
          Serial.println(DRateRoll_tst,2);         
          break;
      default:
        break;
      }  
    #endif      
    }
  #endif  

  // builtin LED flashing when in loopmodus
  if (micros() - LoopTimer2 > 400000) {
  // Reset LoopTimer for the next iteration    
  LoopTimer2 = micros();
  // Toggle LED state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  }
}
