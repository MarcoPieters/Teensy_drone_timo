#include <Arduino.h>

// Include necessary libraries
#include <Wire.h>
#include <PulsePosition.h>

//debug serial print on/off
#define debug

// teensy pinconfiguration
int RecieverPin = 14; //PPM signal reciever
int Motor1Pin = 1; //CCW Front Right
int Motor2Pin = 2; //CW Back Right
int Motor3Pin = 3; //CCW Back Left
int Motor4Pin = 4; //CW Front Left

// Global variables for RC receiver inputs
PulsePositionInput ReceiverInput(RISING);
float ReceiverValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
int ChannelNumber = 0;

// GLobal variables for MotorInput 
int InputRoll;
int InputPitch;
int InputThrottle;
int InputYaw;

// Timer variable for main loop
uint32_t LoopTimer;
uint32_t LoopTimer2;
uint32_t LoopTimer3;

// Array to store PID output and related variables
float PIDReturn[] = {0, 0, 0};

// Variables for Motorinputs
float MotorInput1,MotorInput2,MotorInput3,MotorInput4;

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

void setup() {
  #ifdef debug
   Serial.begin(115200);
  #endif
  // Set pin modes and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   

  // Configure PWM frequencies for motor control
  analogWriteFrequency(Motor1Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor2Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor3Pin, 250); //carrier frequency for PWM signal
  analogWriteFrequency(Motor4Pin, 250); //carrier frequency for PWM signal
  analogWriteResolution(12);  // nr bits resolution

  #ifdef debug
    Serial.println("check channel Throttle for minimium position. Move throttle stick");
  #endif

  // Initialize RC receiver check if throttle is between 1020 and 1200 us  
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
  if (micros() - LoopTimer > 200000){
    // Read receiver inputs
    read_receiver();
  
    // Calculate desired rates based on receiver inputs
    InputRoll = 0.15 * (ReceiverValue[0] - 1500);
    InputPitch = 0.15 * (ReceiverValue[1] - 1500);
    InputThrottle = ReceiverValue[2];
    InputYaw = 0.15 * (ReceiverValue[3] - 1500);

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
    int ThrottleIdle = 1180;
    if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
    if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
    if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
    if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

    // Set cutoff throttle level if RC signal is lost
    int ThrottleCutOff = 1024;
    if (ReceiverValue[2] < 1050) {
      MotorInput1 = ThrottleCutOff; 
      MotorInput2 = ThrottleCutOff;
      MotorInput3 = ThrottleCutOff; 
      MotorInput4 = ThrottleCutOff;
    }

    // Send motor inputs to ESCs
    analogWrite(Motor1Pin, MotorInput1);
    analogWrite(Motor2Pin, MotorInput2);
    analogWrite(Motor3Pin, MotorInput3);
    analogWrite(Motor4Pin, MotorInput4);
  }
  #ifdef debug
    if (micros() - LoopTimer3 > 200000){
    Serial.print("ppm ");
    Serial.print("ch1");
    Serial.print("\t");
    Serial.print(ReceiverValue[0],0);
    Serial.print("\t");
    Serial.print("ch2");
    Serial.print("\t");
    Serial.print(ReceiverValue[1],0);
    Serial.print("\t");
    Serial.print("ch3");
    Serial.print("\t");
    Serial.print(ReceiverValue[2],0);
    Serial.print("\t");
    Serial.print("ch4");
    Serial.print("\t");
    Serial.print(ReceiverValue[3],0);
    /*
    Serial.print("\t");
    Serial.print("ch5");
    Serial.print("\t");
    Serial.print(ReceiverValue[4],0);
    Serial.print("\t");
    Serial.print("ch6");
    Serial.print("\t");
    Serial.print(ReceiverValue[5],0);
    */
    Serial.print("\t");
    Serial.print("Roll");
    Serial.print("\t");
    Serial.print(InputRoll);
    Serial.print("\t");
    Serial.print("Pitch");
    Serial.print("\t");
    Serial.print(InputPitch);
    Serial.print("\t");
    Serial.print("Throttle");
    Serial.print("\t");
    Serial.print(InputThrottle);
    Serial.print("\t");
    Serial.print("Yaw");
    Serial.print("\t");
    Serial.print(InputYaw);
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
  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }
}
