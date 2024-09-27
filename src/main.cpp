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
#include "QMC5883LCompass.h"
#include <TinyGPS++.h>
#include "IBusReceiver.h"
#include "wiring.h"
#include <SoftwareSerial.h>

#define GPS_sensor_active
#define pressure_sensor_active
#define Bluetooth
#define magneto_sensor_active
//#define magneto_calibrate

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
// LED builtin pin 13 used for flash signal that code is in loop.
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
float roll_angle_gyro_fusion, pitch_angle_gyro_fusion,yaw_angle_gyro_fusion;
float roll_angle_acc , pitch_angle_acc;
float velocityX =0.0 , velocityY =0.0, velocityZ=0.0 ;
float positionX =0.0, positionY =0.0, positionZ =0.0; 
int switchB_State;
int switchC_State;

String inputBuffer = "";  // Buffer to store the incoming dat
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
float Voltage, Current, BatteryEnergyPercRemaining, BatteryEnergyAtStart, BatteryEnergyRemaining;
float alpha_current = 0.007;  // Smoothing factor for current, between 0 (smooth) and 1 (fast response)
float alpha_voltage = 0.007;  // Smoothing factor for voltage, between 0 (smooth) and 1 (fast response)
float averagedCurrent = 0.0;
float averagedVoltage = 0.0;
float BatteryEnergyConsumed = 0.0;
float BatteryEnergyDefault4S = 3700.0;  // Capacity of 4S battery in mAh
float BatteryEnergyDefault6S = 4000.0;  // Capacity of 6S battery in mAh
float BatteryEnergyDefault; // To store the selected battery capacity
unsigned long lastUpdateTime = 0;
const float TimeStep = 0.004; // Update period in seconds (4 ms)
int batteryType = 0; // 0 for unknown, 4 for 4S, 6 for 6S

// Global variables for magnetosenso
float magn_x,magn_y,magn_z;
float magn_azimuth,magn_bearing;
char magn_txt_direction[3];
float alpha_magn_azimuth = 0.012;  // Smoothing factor for current, between 0 (smooth) and 1 (fast response)
float averaged_magn_azimuth = 0.0;
boolean isyawInitialized;

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
float PRateRoll,PRateRoll_tst,IRateRoll,IRateRoll_tst,DRateRoll,DRateRoll_tst,PRatePitch,PRatePitch_tst,PRateYaw;
float IRatePitch,IRateYaw,DRateYaw;
float IAngleYaw,DRatePitch,DRatePitch_tst;
float PAngleRoll,PAngleRoll_tst,PAnglePitch,PAnglePitch_tst,PAngleYaw;
float IAngleRoll,IAngleRoll_tst,IAnglePitch,IAnglePitch_tst,DAngleRoll,DAngleRoll_tst,DAnglePitch,DAnglePitch_tst,DAngleYaw;
float PfactorRoll, IfactorRoll, DfactorRoll; 
float PfactorPitch, IfactorPitch, DfactorPitch;
float PfactoreYaw, IfactorYaw, DfactorYaw;

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
float PrevErrorRoll, PrevPtermRoll, PrevItermRoll, PrevDtermRoll;
float PrevErrorPitch, PrevPtermPitch, PrevItermPitch, PrevDtermPitch;
float PrevErrorYaw, PrevPtermYaw, PrevItermYaw, PrevDtermYaw; 

// Variables for desired rates
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw,  InputThrottle;
float DesiredAngleRoll, DesiredAnglePitch, DesiredAngleYaw;

// Variables for pitch yaw roll
float ErrorRateRoll,ErrorRatePitch,ErrorRateYaw;
float ErrorAngleRoll,ErrorAnglePitch,ErrorAngleYaw;
float ErrorRoll,ErrorPitch, ErrorYaw;
float InputRoll,InputPitch,InputYaw;

// Variables for Motorinputs
float MotorInput1,MotorInput2,MotorInput3,MotorInput4;

// Declare Class GyroSignals
GyroSignals gyroSignals;

// Declare Class Barometer
Barometer barometer(0x76);  // BMP280 I2C address

// Declare Class compass
QMC5883LCompass compass;  // QMC5883L 0x0D default I2C address

// Declare GPS sensor
TinyGPSPlus gps;
const uint32_t GPSBaud = 9600;  // GY_GPSV3_NEO_M9N GPSBaud = 38400 : M10A-5883 GPSBaud = 9600
char data;

void battery_voltage(void) 
{
  // Read raw voltage and current from analog pins
  float rawVoltage = (float)analogRead(VoltageBatteryPin) / 40.3;  // Scale to actual voltage
  float rawCurrent = (float)analogRead(CurrentBatteryPin) / 2.0;   // Scale to actual current in amps

  // Apply exponential moving average to smooth the current signal
  averagedCurrent = alpha_current * rawCurrent + (1 - alpha_current) * averagedCurrent;

  // Apply exponential moving average to smooth the voltage signal
  averagedVoltage = alpha_voltage * rawVoltage + (1 - alpha_voltage) * averagedVoltage;

  // Update the global Voltage and Current variables with smoothed values
  Voltage = rawVoltage;
  Current = rawCurrent;
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
    if (voltage >= 16.5) return 100.0;  // Fully charged (4.2V per cell)
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

void battery_check()
{
  // Update battery status every time step
  unsigned long currentTime = millis();
  if ((currentTime - lastUpdateTime) >= TimeStep * 1000)
  {
    lastUpdateTime = currentTime;

    // Read voltage and current
    battery_voltage();

    // Integrate current to update energy consumption (Coulomb Counting)
    BatteryEnergyConsumed += Current * 1000.0 * TimeStep / 3600.0; // mAh used

    // Calculate percentage of remaining energy and absoluter remaining energy
    BatteryEnergyPercRemaining = (BatteryEnergyAtStart - BatteryEnergyConsumed) / BatteryEnergyDefault * 100.0;
    BatteryEnergyRemaining = (BatteryEnergyAtStart - BatteryEnergyConsumed);

    // If battery is idle, update the SoC based on voltage (idle reading)
    if (abs(Current) < 0.3) // If the current is very small, consider battery at idle
    {
      BatteryEnergyPercRemaining = estimate_capacity_from_voltage(Voltage, batteryType);
    }

    // Control LED based on battery Energylevel percentage
    if (BatteryEnergyPercRemaining <= 30.0)
      digitalWrite(LedRedPin, HIGH);
    else
      digitalWrite(LedRedPin, LOW);
  }
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
  PrevErrorRoll = 0;
  PrevErrorPitch = 0;
  PrevErrorYaw = 0;
  PrevPtermRoll = 0;
  PrevPtermPitch = 0;
  PrevPtermYaw = 0;
  PrevItermRoll = 0;
  PrevItermPitch = 0;
  PrevItermYaw = 0;
  PrevDtermRoll = 0;
  PrevDtermPitch = 0;
  PrevDtermYaw = 0;

}

void serial_print_app( Stream &SERIAL_OUTPUT = Serial8){
      SERIAL_OUTPUT.print("*G");
      SERIAL_OUTPUT.print(roll_angle_gyro_fusion,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(pitch_angle_gyro_fusion,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(yaw_angle_gyro,0);
      SERIAL_OUTPUT.println("*");
      SERIAL_OUTPUT.print("*H");
      SERIAL_OUTPUT.print(RateRoll,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(RatePitch,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(RateYaw,0);
      SERIAL_OUTPUT.println("*");
      SERIAL_OUTPUT.print("*I");
      SERIAL_OUTPUT.print(AccX,1);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(AccY,1);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(AccZ,1);
      SERIAL_OUTPUT.println("*");
      SERIAL_OUTPUT.print("*J");
      SERIAL_OUTPUT.print(magn_x,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(magn_y,0);
      SERIAL_OUTPUT.print(",");
      SERIAL_OUTPUT.print(magn_z,0);
      SERIAL_OUTPUT.println("*");
      SERIAL_OUTPUT.print("*A");
      SERIAL_OUTPUT.print(pressure,0);
      SERIAL_OUTPUT.println("*");
      SERIAL_OUTPUT.print("*B");
      SERIAL_OUTPUT.print(relativeAltitude,0);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*C");
      SERIAL_OUTPUT.print(cTemp,0);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*D");
      SERIAL_OUTPUT.print(averagedVoltage,1);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*E");
      SERIAL_OUTPUT.print(averagedCurrent,1);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*F");
      SERIAL_OUTPUT.print(velocityX,3);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*K");
      SERIAL_OUTPUT.print(positionX,3);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*L");
      SERIAL_OUTPUT.print(AccX,3);
      SERIAL_OUTPUT.println("*"); 
      SERIAL_OUTPUT.print("*M");
      SERIAL_OUTPUT.print(yaw_angle_gyro_fusion,1);
      SERIAL_OUTPUT.println("*");
}

void serial_print_debug( Stream &SERIAL_OUTPUT = Serial8){
    #ifdef debug_teleplot_graph
      #ifdef debug_receiver
        for (int i = 1; i <= IBUS_CHANNELS; i++) 
          {
          SERIAL_OUTPUT.print(">Ch");
          SERIAL_OUTPUT.print(i);
          SERIAL_OUTPUT.print(":");
          SERIAL_OUTPUT.print(ReceiverValue[i-1]);
          SERIAL_OUTPUT.print("\n");
          }
      #endif
      #ifdef debug_barometer
        SERIAL_OUTPUT.print(">Temp:");
        SERIAL_OUTPUT.print(cTemp);
        SERIAL_OUTPUT.print("\n");
        SERIAL_OUTPUT.print(">Press:");
        SERIAL_OUTPUT.print(pressure, 3);
        SERIAL_OUTPUT.print("\n");
        SERIAL_OUTPUT.print(">Alt:");
        SERIAL_OUTPUT.println(relativeAltitude, 0);
      #endif
      #ifdef magneto_sensor_active
        Serial.print(">Mag_x:");
        Serial.println(magn_x);
        Serial.print(">Mag_y:");
        Serial.println(magn_y);
        Serial.print(">Mag_z:");
        Serial.println(magn_z);
        Serial.print(">Mag_azimuth:");
        Serial.println(magn_azimuth);
        Serial.print(">Mag_avg_azimuth:");
        Serial.println(averaged_magn_azimuth);
        Serial.print(">Mag_bearing:");
        Serial.println(magn_bearing);
        Serial.print(">Yaw_angel:");
        Serial.println(yaw_angle_gyro_fusion); 
      #endif  
      #ifdef debug_GPS
        while (Serial2.available() > 0) {
        data = Serial2.read();
        if (gps.encode(data)) {
          if (gps.location.isValid()) {
            SERIAL_OUTPUT.print(">Lat:");
            SERIAL_OUTPUT.println(gps.location.lat(), 6);
            SERIAL_OUTPUT.print(">Long:");
            SERIAL_OUTPUT.println(gps.location.lng(), 6);
          } else {
            SERIAL_OUTPUT.println("Location: Not Available");
          }
          if (gps.altitude.isValid()) {
            SERIAL_OUTPUT.print(">Alt_gps:");
            SERIAL_OUTPUT.println(gps.altitude.meters());
          } else {
            SERIAL_OUTPUT.println("Altitude: Not Available");
          }
          if (gps.speed.isValid()) {
            SERIAL_OUTPUT.print(">V_gps:");
            SERIAL_OUTPUT.println(gps.speed.kmph());
          } else {
            SERIAL_OUTPUT.println("Speed: Not Available");
          }
          if (gps.satellites.isValid()) {
            SERIAL_OUTPUT.print(">Sat_nr_gps:");
            SERIAL_OUTPUT.println(gps.satellites.value());
          } else {
            SERIAL_OUTPUT.println("Number of Satellites: Not Available");
          }
          if (gps.course.isValid()) {
            SERIAL_OUTPUT.print(">Course_(heading)_gps:");
            SERIAL_OUTPUT.println(gps.course.deg());
          } else {
            SERIAL_OUTPUT.println("Course (heading): Not Available");
          }
          if (gps.hdop.isValid()) {
            SERIAL_OUTPUT.print(">HDOP:");//horizontal dilution of precision 
            SERIAL_OUTPUT.println(gps.hdop.value());
          } else {
            SERIAL_OUTPUT.println("hdop: Not Available");
          }
          if (gps.date.isValid()) {
            SERIAL_OUTPUT.print("Date_UTC:");
            SERIAL_OUTPUT.print(gps.date.month());
            SERIAL_OUTPUT.print("/");
            SERIAL_OUTPUT.print(gps.date.day());
            SERIAL_OUTPUT.print("/");
            SERIAL_OUTPUT.println(gps.date.year());
          } else {
            SERIAL_OUTPUT.println("Date: Not Available");
          }
          if (gps.time.isValid()) {
            SERIAL_OUTPUT.print("Time_UTC:");
            if (gps.time.hour() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.print(gps.time.hour());
            SERIAL_OUTPUT.print(":");
            if (gps.time.minute() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.print(gps.time.minute());
            SERIAL_OUTPUT.print(":");
            if (gps.time.second() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.println(gps.time.second());
            SERIAL_OUTPUT.print(">Time_UTC:");
            SERIAL_OUTPUT.println(gps.time.value());
          } else {
            SERIAL_OUTPUT.println("Time: Not Available");
          }
        }
        }
      #endif
      SERIAL_OUTPUT.print(">V:");
      SERIAL_OUTPUT.println(Voltage,2);
      SERIAL_OUTPUT.print(">V_avg:");
      SERIAL_OUTPUT.println(averagedVoltage,1);
      SERIAL_OUTPUT.print(">I:");
      SERIAL_OUTPUT.println(Current,2);
      SERIAL_OUTPUT.print(">I_avg:");
      SERIAL_OUTPUT.println(averagedCurrent,1);
      SERIAL_OUTPUT.print(">R_A_roll:");
      SERIAL_OUTPUT.println(roll_angle_gyro_fusion,0);
      SERIAL_OUTPUT.print(">R_A_pitch:");
      SERIAL_OUTPUT.println(pitch_angle_gyro_fusion,0);
      SERIAL_OUTPUT.print(">R_A_yaw:");
      SERIAL_OUTPUT.println(yaw_angle_gyro,0);
      SERIAL_OUTPUT.print(">R_R_roll:");
      SERIAL_OUTPUT.println(RateRoll,0);
      SERIAL_OUTPUT.print(">R_R_pitch:");
      SERIAL_OUTPUT.println(RatePitch,0);
      SERIAL_OUTPUT.print(">R_R_yaw:");
      SERIAL_OUTPUT.println(RateYaw,0);
      SERIAL_OUTPUT.print(">D_A_roll:");
      SERIAL_OUTPUT.println(DesiredAngleRoll,0);
      SERIAL_OUTPUT.print(">D_A_pitch:");
      SERIAL_OUTPUT.println(DesiredAnglePitch,0);
      SERIAL_OUTPUT.print(">D_R_yaw:");
      SERIAL_OUTPUT.println(DesiredRateYaw,0);
      SERIAL_OUTPUT.print(">D_R_roll:");
      SERIAL_OUTPUT.println(DesiredRateRoll,0);
      SERIAL_OUTPUT.print(">D_R_pitch:");
      SERIAL_OUTPUT.println(DesiredRatePitch,0);
      SERIAL_OUTPUT.print(">D_R_yaw:");        
      SERIAL_OUTPUT.println(DesiredRateYaw,0);
      SERIAL_OUTPUT.print(">D_power:");
      SERIAL_OUTPUT.println(InputThrottle,0);
      SERIAL_OUTPUT.print(">E_A_roll:");
      SERIAL_OUTPUT.println(ErrorRoll,0);
      SERIAL_OUTPUT.print(">E_A_pitch:  ");
      SERIAL_OUTPUT.println(ErrorPitch,0);
      SERIAL_OUTPUT.print(">E_R_yaw:");
      SERIAL_OUTPUT.println(ErrorYaw,0);
      SERIAL_OUTPUT.print(">P_roll:");
      SERIAL_OUTPUT.println(PrevPtermRoll,0);
      SERIAL_OUTPUT.print(">I_roll:");
      SERIAL_OUTPUT.println(PrevItermRoll,0);
      SERIAL_OUTPUT.print(">D_roll:");
      SERIAL_OUTPUT.println(PrevDtermRoll,0);
      SERIAL_OUTPUT.print(">In_roll:");
      SERIAL_OUTPUT.println(InputRoll,0);
      SERIAL_OUTPUT.print(">P_pitch:");
      SERIAL_OUTPUT.println(PrevPtermPitch,0);
      SERIAL_OUTPUT.print(">I_pitch:");
      SERIAL_OUTPUT.println(PrevItermPitch,0);
      SERIAL_OUTPUT.print(">D_pitch:");
      SERIAL_OUTPUT.println(PrevDtermPitch,0);
      SERIAL_OUTPUT.print(">In_pitch:");
      SERIAL_OUTPUT.println(InputPitch,0);
      SERIAL_OUTPUT.print(">P_yaw:");
      SERIAL_OUTPUT.println(PrevPtermYaw,0);
      SERIAL_OUTPUT.print(">I_yaw:");
      SERIAL_OUTPUT.println(PrevItermYaw,0);
      SERIAL_OUTPUT.print(">D_yaw:");
      SERIAL_OUTPUT.println(PrevDtermYaw,0);
      SERIAL_OUTPUT.print(">In_yaw:  ");
      SERIAL_OUTPUT.println(InputYaw,0);
      SERIAL_OUTPUT.print(">M4:");
      SERIAL_OUTPUT.println(MotorInput4,0);
      SERIAL_OUTPUT.print(">M3:");
      SERIAL_OUTPUT.println(MotorInput3,0);
      SERIAL_OUTPUT.print(">M2:");
      SERIAL_OUTPUT.println(MotorInput2,0);
      SERIAL_OUTPUT.print(">M1:");
      SERIAL_OUTPUT.println(MotorInput1,0);
      SERIAL_OUTPUT.print(">PrateR:");
      SERIAL_OUTPUT.println(PfactorRoll,1);
      SERIAL_OUTPUT.print(">IrateR:");
      SERIAL_OUTPUT.println(IfactorRoll,1);
      SERIAL_OUTPUT.print(">DrateR:");
      SERIAL_OUTPUT.println(DfactorRoll,2); 
      SERIAL_OUTPUT.print(">SWB:");
      SERIAL_OUTPUT.println(switchB_State);
      SERIAL_OUTPUT.print(">SWC:");
      SERIAL_OUTPUT.println(switchC_State);
      SERIAL_OUTPUT.print(">BattStart:");
      SERIAL_OUTPUT.println(BatteryEnergyAtStart,0); 
      SERIAL_OUTPUT.print(">BattConsumed:");
      SERIAL_OUTPUT.println(BatteryEnergyConsumed,2);
      SERIAL_OUTPUT.print(">BattRemain:");
      SERIAL_OUTPUT.println(BatteryEnergyRemaining,0);
      SERIAL_OUTPUT.print(">BattRemainPerc:");
      SERIAL_OUTPUT.println(BatteryEnergyPercRemaining,0);
      SERIAL_OUTPUT.print(">VelocX:");
      SERIAL_OUTPUT.println(velocityX,2);
      SERIAL_OUTPUT.print(">PosX:");
      SERIAL_OUTPUT.println(positionX,2);
    #endif 

    #ifdef debug_plot_graph
      #ifdef debug_receiver
      for (int i = 1; i <= IBUS_CHANNELS; i++) {
              SERIAL_OUTPUT.print("Ch");
              SERIAL_OUTPUT.print(i);
              SERIAL_OUTPUT.print(": ");
              SERIAL_OUTPUT.print(ReceiverValue[i-1]);
              SERIAL_OUTPUT.print(" ");
      }
      SERIAL_OUTPUT.println();
      #endif
      #ifdef debug_barometer
        SERIAL_OUTPUT.print("Temperature: ");
        SERIAL_OUTPUT.print(cTemp);
        SERIAL_OUTPUT.print(" C\tPressure: ");
        SERIAL_OUTPUT.print(pressure, 3);
        SERIAL_OUTPUT.print(" mbar\trelativeAltitude: ");
        SERIAL_OUTPUT.print(relativeAltitude, 0);
        SERIAL_OUTPUT.println(" cm");
      #endif 
      #ifdef debug_GPS
        while (Serial2.available() > 0) {
        data = Serial2.read();
        if (gps.encode(data)) {
          if (gps.location.isValid()) {
            SERIAL_OUTPUT.print("Latitude : ");
            SERIAL_OUTPUT.println(gps.location.lat(), 6);
            SERIAL_OUTPUT.print("Longitude: ");
            SERIAL_OUTPUT.println(gps.location.lng(), 6);
          } else {
            SERIAL_OUTPUT.println("Location: Not Available");
          }
          if (gps.altitude.isValid()) {
            SERIAL_OUTPUT.print("Altitude: ");
            SERIAL_OUTPUT.print(gps.altitude.meters());
            SERIAL_OUTPUT.println(" meters");
          } else {
            SERIAL_OUTPUT.println("Altitude: Not Available");
          }
          if (gps.speed.isValid()) {
            SERIAL_OUTPUT.print("Speed: ");
            SERIAL_OUTPUT.print(gps.speed.kmph());
            SERIAL_OUTPUT.println(" km/h");
          } else {
            SERIAL_OUTPUT.println("Speed: Not Available");
          }
          if (gps.satellites.isValid()) {
            SERIAL_OUTPUT.print("Number of Satellites: ");
            SERIAL_OUTPUT.println(gps.satellites.value());
          } else {
            SERIAL_OUTPUT.println("Number of Satellites: Not Available");
          }
          if (gps.course.isValid()) {
            SERIAL_OUTPUT.print("Course (heading): ");
            SERIAL_OUTPUT.print(gps.course.deg());
            SERIAL_OUTPUT.println(" degrees");
          } else {
            SERIAL_OUTPUT.println("Course (heading): Not Available");
          }
          if (gps.hdop.isValid()) {
            SERIAL_OUTPUT.print("HDOP horizontal dilution of precision : ");
            SERIAL_OUTPUT.print(gps.hdop.value());
            SERIAL_OUTPUT.println(" ");
          } else {
            SERIAL_OUTPUT.println("hdop: Not Available");
          }
          if (gps.date.isValid()) {
            SERIAL_OUTPUT.print("Date UTC: ");
            SERIAL_OUTPUT.print(gps.date.month());
            SERIAL_OUTPUT.print("/");
            SERIAL_OUTPUT.print(gps.date.day());
            SERIAL_OUTPUT.print("/");
            SERIAL_OUTPUT.println(gps.date.year());
          } else {
            SERIAL_OUTPUT.println("Date: Not Available");
          }
          if (gps.time.isValid()) {
            SERIAL_OUTPUT.print("Time UTC: ");
            if (gps.time.hour() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.print(gps.time.hour());
            SERIAL_OUTPUT.print(":");
            if (gps.time.minute() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.print(gps.time.minute());
            SERIAL_OUTPUT.print(":");
            if (gps.time.second() < 10) SERIAL_OUTPUT.print("0");
            SERIAL_OUTPUT.println(gps.time.second());
          } else {
            SERIAL_OUTPUT.println("Time: Not Available");
          }
          SERIAL_OUTPUT.println();
        }
        }
      #endif      
      switch (steering_manner)
      {
        case 1:
          SERIAL_OUTPUT.print("R_Roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(RateRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("R_Pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(RatePitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("R_Yaw:  ");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(RateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_R_roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredRateRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("Angle_Roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(roll_angle_gyro_fusion,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("Angle_Pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(pitch_angle_gyro_fusion,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_R_pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredRatePitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_R_yaw:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredRateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_power:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(InputThrottle,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_R_roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorRateRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_R_pitch:  ");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorRatePitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_R_yaw:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorRateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M4:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput4,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M3:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput3,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M2:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput2,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M1:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.println(MotorInput1,0);
          break;
        case 2:
          SERIAL_OUTPUT.print("V:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(Voltage,2);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("I:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(Current,2);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("R_A_Roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(roll_angle_gyro_fusion,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("R_A_Pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(pitch_angle_gyro_fusion,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("R_R_Yaw:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(RateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_A_roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredAngleRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_A_pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredAnglePitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_R_yaw:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(DesiredRateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("D_power:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(InputThrottle,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_A_roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorAngleRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_A_pitch:  ");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorAnglePitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("E_R_yaw:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(ErrorRateYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("I_Roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(PrevItermAngleRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("In_Roll:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(InputRoll,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("In_Pitch:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(InputPitch,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("In_Yaw:  ");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(InputYaw,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M4:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput4,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M3:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput3,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M2:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput2,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("M1:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(MotorInput1,0);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("PrateR:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(PRateRoll_tst,1);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("IrateR:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print(IRateRoll_tst,1);
          SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.print("DrateR:");
          //SERIAL_OUTPUT.print("\t");
          SERIAL_OUTPUT.println(DRateRoll_tst,2);         
          break;
      default:
        break;
      }  
    #endif      
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
  
  #ifdef Bluetooth
    Serial8.begin(115200);

    Serial.println("Bluetooth HC-05 module connected to Serial5 on Teensy 4.1");
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
  Wire.setClock(400000);//400000
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

  #ifdef magneto_sensor_active
    compass.init();
    // magneto compass reading
    compass.read();
    averaged_magn_azimuth = compass.getAzimuth();

    delay(1000);

    Serial.println("CALIBRATING. Keep moving your sensor...");
    compass.calibrate();

    #ifdef magneto_calibrate
      delay(2000);
      Serial.println("DONE. Copy the lines below and paste it into your projects sketch.);");
      Serial.println();
      Serial.print("compass.setCalibrationOffsets(");
      Serial.print(compass.getCalibrationOffset(0));
      Serial.print(", ");
      Serial.print(compass.getCalibrationOffset(1));
      Serial.print(", ");
      Serial.print(compass.getCalibrationOffset(2));
      Serial.println(");");
      Serial.print("compass.setCalibrationScales(");
      Serial.print(compass.getCalibrationScale(0));
      Serial.print(", ");
      Serial.print(compass.getCalibrationScale(1));
      Serial.print(", ");
      Serial.print(compass.getCalibrationScale(2));
      Serial.println(");");
      delay(2000);
    #endif
    compass.setCalibrationOffsets(-421.0,-252.0,490);
    compass.setCalibrationScales(0.94,0.9,1.2);
    

  #endif

  // Configure PWM frequencies for motor control
  analogWriteFrequency(Motor1Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor2Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor3Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor4Pin, 250); //carrier frequency for PWM signal
  analogWriteResolution(12);  // nr bits resolution
 
  // Initialize the starting battery energy based on voltage
  battery_voltage();  // Initial voltage reading
  detect_battery_type(Voltage);  // Detect if 4S or 6S battery is connected
  BatteryEnergyAtStart = BatteryEnergyDefault * estimate_capacity_from_voltage(Voltage, batteryType) / 100.0;
  lastUpdateTime = millis();

  #ifndef IBUS_READ
  // Initialize RC receiver
  ReceiverInput.begin(RecieverPin);
  #endif
  
   // Set status setup ready
  pinMode(LedGreenPin, OUTPUT);
  digitalWrite(LedGreenPin, HIGH);
  
  //while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1200) {
  //  read_receiver();
  //  delay(4);
  //}
  //Default PID settings Rate
  PRateRoll = 0.6;  //0.6
  IRateRoll = 3.5; // 3.5
  DRateRoll = 0.03;
  PRatePitch = PRateRoll;
  IRatePitch = IRateRoll;
  DRatePitch = DRateRoll;
  PRateYaw = 2;
  IRateYaw = 12;
  DRateYaw = 0;
  
  //Default PID settings Angel
  PAngleRoll = 1.3; // 1.3 :resonantie op 10
  IAngleRoll = 1.1; // 3.3
  DAngleRoll = 0.2; //0.28
  PAnglePitch = 1.1; //resonantie op 10
  IAnglePitch = 1.1;
  DAnglePitch = 0.2;
  PAngleYaw = 2;
  IAngleYaw = 0.0;
  DAngleYaw = 0.0;

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
    {
    // sensor read every 4ms
    if (micros() - LoopTimer4 > 4000){
        //Serial.print(micros() - LoopTimer4);
        //Serial.print(" ");
        LoopTimer4 = micros();
        
        #ifdef magneto_sensor_active
        // magneto compass reading
        compass.read();

        magn_x = compass.getX();
        magn_y = compass.getY();
        magn_z = compass.getZ();
        magn_azimuth = compass.getAzimuth();
        // Apply exponential moving average to smooth the magneto azimuth signal
        averaged_magn_azimuth = alpha_magn_azimuth * magn_azimuth + (1 - alpha_magn_azimuth) * averaged_magn_azimuth;
		    magn_bearing = compass.getBearing(magn_azimuth);
          // Step 3: Initialize the yaw_angle to the first magnetometer reading
        if (!isyawInitialized) {
          yaw_angle_gyro = magn_azimuth;   // Set the initial yaw angle to the magnetometer's azimuth
          isyawInitialized = true;    // Mark as initialized
        }
        #endif

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
        yaw_angle_gyro_fusion += RateYaw * time_difference;

        // Calculate roll angle in degrees
        roll_angle_acc = atan2(AccY, sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / PI;

        // Calculate pitch angle in degrees
        pitch_angle_acc = atan2(-AccX, sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / PI;

        // sensor fusion
        // sensor fusion with complementary filter gyro and accelerator sensor
        #ifdef sensor_fusion 
          roll_angle_gyro_fusion = 0.98 * roll_angle_gyro_fusion + 0.02 * roll_angle_acc;
          pitch_angle_gyro_fusion = 0.98 * pitch_angle_gyro_fusion + 0.02 * pitch_angle_acc;
          yaw_angle_gyro_fusion = 0.98 * yaw_angle_gyro_fusion + 0.02 * averaged_magn_azimuth;
        #endif
        
        // Convert roll, pitch, yaw to radians for rotation matrix
        float roll_rad = roll_angle_gyro_fusion * PI / 180.0;
        float pitch_rad = pitch_angle_gyro_fusion* PI / 180.0;
        float yaw_rad = yaw_angle_gyro * PI / 180.0;

        // Integrate acceleration to get velocity
        velocityX += AccX * time_difference;
        velocityY += AccY * time_difference;
        velocityZ += AccZ * time_difference;

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
      switchB_State = determine3SwitchState(ReceiverValue[8]); 
      switchC_State = determine3SwitchState(ReceiverValue[9]);
      switch (switchB_State)
      {
      case 1: // fixed PID parameters
        // Calculate desired rates based on receiver inputs
        DesiredAngleRoll = 0.07 * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.07 * (ReceiverValue[1] - 1500);
        InputThrottle = ReceiverValue[2];
        DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

        PfactorRoll = PAngleRoll;
        IfactorRoll = IAngleRoll;
        DfactorRoll = DAngleRoll;
        PfactorPitch = PAnglePitch;
        IfactorPitch = IAnglePitch;
        DfactorPitch = DAnglePitch;
        PfactoreYaw = PAngleYaw;
        IfactorYaw = IAngleYaw;
        DfactorYaw = DAngleYaw;

        // Calculate errors for PID control
        ErrorRoll = DesiredAngleRoll - roll_angle_gyro_fusion;
        ErrorPitch = DesiredAnglePitch - pitch_angle_gyro_fusion;
        ErrorYaw = DesiredRateYaw - RateYaw;

        break;
      case 2: // adjustable PID parameters Roll      
        // Calculate desired rates based on receiver inputs
        DesiredAngleRoll = 0.07 * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.07 * (ReceiverValue[1] - 1500);
        InputThrottle = ReceiverValue[2];
        DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);  
        switch (switchC_State)
        {
        case 1: //change Proportional parameter setting
            // Smoothly update P gain
            PAngleRoll_tst = 0.9 * PAngleRoll_tst + 0.1 * (0.0025 * (ReceiverValue[4]- 1000));
            // Ensure IAngleRoll_tst does not go below 0
            PAngleRoll_tst = max(PAngleRoll_tst, 0.0);
            PfactorRoll = PAngleRoll_tst;
            PfactorPitch = PAnglePitch;
          break;
        case 2: // change Intergral parameter setting
            //Smoothly update I gain
            IAngleRoll_tst = 0.9 * IAngleRoll_tst + 0.1 * (0.01 * (ReceiverValue[4] - 1000));
            // Ensure IAngleRoll_tst does not go below 0
            IAngleRoll_tst = max(IAngleRoll_tst, 0.0);
            IfactorRoll = IAngleRoll_tst;
            IfactorPitch = IAnglePitch;
          break;
        case 3: // change differential parameter
            //Smoothly update D gain;
            DAngleRoll_tst = 0.9 * DAngleRoll_tst + 0.1 * (0.001 * (ReceiverValue[4] - 1005));
            // Ensure DAngleRoll_tst does not go below 0
            DAngleRoll_tst = max(DAngleRoll_tst, 0.0);
            DfactorRoll = DAngleRoll_tst;
            DfactorPitch = DAnglePitch;
          break;  
        default:
          break;
        }
        PfactoreYaw = PAngleYaw;
        IfactorYaw = IAngleYaw;
        DfactorYaw = DAngleYaw;
        // Calculate errors for PID control
        ErrorRoll = DesiredAngleRoll - roll_angle_gyro_fusion;
        ErrorPitch = DesiredAnglePitch - pitch_angle_gyro_fusion;
        ErrorYaw = DesiredRateYaw - RateYaw;
        break;
      case 3:  // rate steering manner
        // Calculate desired rates based on receiver inputs
        DesiredRateRoll = 0.15 * (ReceiverValue[0] - 1500);
        DesiredRatePitch = 0.15 * (ReceiverValue[1] - 1500);
        InputThrottle = ReceiverValue[2];
        DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

        PfactorRoll = PRateRoll;
        IfactorRoll = IRateRoll;
        DfactorRoll = DRateRoll;
        PfactorPitch = PRatePitch;
        IfactorPitch = IRatePitch;
        DfactorPitch = DRatePitch;
        PfactoreYaw = PRateYaw;
        IfactorYaw = IRateYaw;
        DfactorYaw = DRateYaw;

        // Calculate errors for PID control
        ErrorRoll = DesiredRateRoll - RateRoll;
        ErrorPitch = DesiredRatePitch - RatePitch;
        ErrorYaw = DesiredRateYaw - RateYaw;
        break;      
      default:
        break;
      }
      // Apply PID control for roll
      pid_equation(ErrorRoll, PfactorRoll, IfactorRoll, DfactorRoll, PrevErrorRoll, PrevItermRoll);
      InputRoll = PIDReturn[0];
      PrevErrorRoll = PIDReturn[1];
      PrevItermRoll = PIDReturn[2];
      PrevPtermRoll = PIDReturn[3];
      PrevDtermRoll = PIDReturn[4];

      // Apply PID control for pitch
      pid_equation(ErrorPitch, PfactorPitch, IfactorPitch, DfactorPitch, PrevErrorPitch, PrevItermPitch);
      InputPitch = PIDReturn[0];
      PrevErrorPitch = PIDReturn[1];
      PrevItermPitch = PIDReturn[2];
      PrevPtermPitch = PIDReturn[3];
      PrevDtermPitch = PIDReturn[4];

      // Apply PID control for yaw  !!! rate not angle !!!!
      pid_equation(ErrorYaw, PfactoreYaw, IfactorYaw, DfactorYaw, PrevErrorYaw, PrevItermYaw);
      InputYaw = PIDReturn[0];
      PrevErrorYaw = PIDReturn[1];
      PrevItermYaw = PIDReturn[2];
      PrevPtermYaw = PIDReturn[3];
      PrevDtermYaw = PIDReturn[4];

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
    analogWrite(Motor1Pin, MotorInput1); //CCW Front Right
    analogWrite(Motor2Pin, MotorInput2); //CW Back Right
    analogWrite(Motor3Pin, MotorInput3); //CCW Back Left
    analogWrite(Motor4Pin, MotorInput4); //CW Front Left

    // check the energy of the battery
    battery_check();
    }

  // print debug values to USB every 100ms
  #ifdef debug
    if (micros() - LoopTimer3 > 100000){
    LoopTimer3 = micros();
    serial_print_debug(Serial);
    #ifdef Bluetooth
      //serial_print_debug(Serial8);
      serial_print_app(Serial8);
    #endif  
    }
  #endif  

while (Serial5.available() > 0) {
    char incomingChar = Serial5.read();  // Read one character from Serial5
    
    // If the character is the separator '*', process the buffer
    if (incomingChar == '*') {
      // Process the buffer (you can print or decode the data)
      Serial.println("Received Data: " + inputBuffer);  // Send the data to Serial
      inputBuffer = "";  // Clear the buffer after processing
    } else {
      // Append the character to the buffer if it's not the separator
      inputBuffer += incomingChar;
    }
  }

  // builtin LED flashing when in loopmodus
  if (micros() - LoopTimer2 > 400000) {
  // Reset LoopTimer for the next iteration    
  LoopTimer2 = micros();
  // Toggle LED state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  }
}

