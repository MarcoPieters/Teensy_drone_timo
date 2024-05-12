#include <Arduino.h>

// Include necessary libraries
#include <Wire.h>

//debug serial print on/off
#define debug

// BMP280 baro registers
int device_address_BMP280 = 0x76 ; 
int CTRL_meas = 0xF4;
int config = 0xF5;
float initialPressure; // Variable to store the initial pressure reading

// Timer variable for main loop
uint32_t LoopTimer;
uint32_t LoopTimer2;
uint32_t LoopTimer3;
uint32_t current_time;
float time_difference;
uint32_t previous_time;

#include <Wire.h>

struct BaroData {
    float pressure;
    float cTemp;
};

float calculateAltitude(float pressure) {
    const float seaLevelPressure = 101325.0; // Sea level pressure in pascals
    float altitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903)) * 100; // altitude in cm
    return altitude;
}

BaroData baro_signals() {
    BaroData data;

    // Read data from 0x88(136), 24 bytes
    Wire.beginTransmission(device_address_BMP280);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(device_address_BMP280, 24);

    byte b1[24];
    for(int i = 0; i < 24; i++) {
        b1[i] = Wire.read();
    }

    // Convert the data
    // Temp coefficients
    u_int16_t dig_T1 = (b1[1] << 8) | b1[0];
    int16_t dig_T2 = (b1[3] << 8) | b1[2];
    int16_t dig_T3 = (b1[5] << 8) | b1[4];
 
    // Convert pressure coefficients
    u_int16_t dig_P1 = (b1[7] << 8) | b1[6];
    int16_t dig_P2 = (b1[9] << 8) | b1[8];
    int16_t dig_P3 = (b1[11] << 8) | b1[10];
    int16_t dig_P4 = (b1[13] << 8) | b1[12];
    int16_t dig_P5 = (b1[15] << 8) | b1[14];
    int16_t dig_P6 = (b1[17] << 8) | b1[16];
    int16_t dig_P7 = (b1[19] << 8) | b1[18];
    int16_t dig_P8 = (b1[21] << 8) | b1[20];
    int16_t dig_P9 = (b1[23] << 8) | b1[22];

    // Read data from 0xF7(247), 8 bytes
    Wire.beginTransmission(device_address_BMP280);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(device_address_BMP280, 8);

    byte data_buffer[8];
    for(int i = 0; i < 8; i++) {
        data_buffer[i] = Wire.read();
    }

    // Convert pressure and temperature data to 19-bits
    uint32_t adc_p = ((uint32_t)data_buffer[0] << 16) | ((uint32_t)data_buffer[1] << 8) | (uint32_t)(data_buffer[2] & 0xF0);
    adc_p /= 16;
    uint32_t adc_t = ((uint32_t)data_buffer[3] << 16) | ((uint32_t)data_buffer[4] << 8) | (uint32_t)(data_buffer[5] & 0xF0);
    adc_t /= 16;


    // Temperature offset calculations
    float var1 = (double(adc_t) / 16384.0 - double(dig_T1) / 1024.0) * double(dig_T2);
    float var2 = ((double(adc_t) / 131072.0 - double(dig_T1) / 8192.0) * (double(adc_t) / 131072.0 - double(dig_T1) / 8192.0)) * double(dig_T3);
    float t_fine = var1 + var2;
    float cTemp = t_fine / 5120.0;

    // Pressure offset calculations
    float var3 = (t_fine / 2.0) - 64000.0;
    float var4 = var3 * var3 * double(dig_P6) / 32768.0;
    var4 = var4 + var3 * double(dig_P5) * 2.0;
    var4 = (var4 / 4.0) + (double(dig_P4) * 65536.0);
    var3 = (double(dig_P3) * var3 * var3 / 524288.0 + double(dig_P2) * var3) / 524288.0;
    var3 = (1.0 + var3 / 32768.0) * (dig_P1);
    float p = 1048576.0 - adc_p;
    p = (p - (var4 / 4096.0)) * 6250.0 / var3;
    float var5 = double(dig_P9) * p * p / 2147483648.0;
    float var6 = p * double(dig_P8) / 32768.0;
    float pressure = (p + (var5 + var6 + double(dig_P7)) / 16.0) / 100.0;

    data.pressure = pressure;
    data.cTemp = cTemp;
    
    return data;
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
    Serial.println("Begin transmission BMP280"); 
  #endif

// BMP280 address, 0x76(118)
//Select Control measurement register, 0xF4(244)
// value 0x27 Pressure 0b001xxxxx and Temperature 0bxxx001xx Oversampling rate = 1 powermode 0bxxxxxx11 = normal mode
// oversampling 000 = 0x 001 = 1x; 010 = 2x; 011 = 4x; 100 = 8x; 101 = 16x
// value 0xB8 Pressure 0b101xxxxx Oversampling rate = 16 and Temperature 0bxxx010xx Oversampling rate = 2 powermode 0bxxxxxx11 = normal mode
  Wire.beginTransmission(device_address_BMP280);
  Wire.write(CTRL_meas);
  Wire.write(0b10101011); // Configure for normal operation P Oversampling rate = 16: T Oversampling rate = 2
  Wire.endTransmission();


// BMP280 address, 0x76(118)
// Select Configuration register, 0xF5(245)
// time 000 = 1ms; 001 = 63ms; 010 = 125ms; 011 = 250ms; 100 = 500ms; 101 = 1000ms; 110 = 2000ms; 111 = 4000ms
// 0b000xxxxx inactive time. 0xA0(00) Stand_by time = 1000 ms
// 0bxxx000xx Filter coefficient.  0bxxx000xx = OFF  0bxxx001xx=2 0bxxx010xx=4 0bxxx011xx=8 0bxxx100xx=16
// 0bxxxxxxx0 SPI mode
  Wire.beginTransmission(device_address_BMP280);
  Wire.write(config);
  Wire.write(0b10101100); // Configure for normal operation
  Wire.endTransmission();

  delay(500);


  #ifdef debug 
    Serial.println("Init BMP280 successfull");
  #endif
  
  // Read initial pressure
  BaroData data = baro_signals();
  initialPressure = data.pressure;

  // Start loop timer
  LoopTimer = micros();
  LoopTimer2 = micros();
  LoopTimer3 = micros();
  previous_time = micros();

}

void loop() {
  if (micros() - LoopTimer > 1000000){
    // Read barometer data
    BaroData data = baro_signals();
    // Access the pressure and temperature values from the BaroData struct
    float pressure = data.pressure; // Pressure value in hPa
    float cTemp = data.cTemp;       // Temperature value in Celsius
   // Calculate altitude relative to initial pressure
    float relativeAltitude = calculateAltitude(pressure) - calculateAltitude(initialPressure);

    #ifdef debug
      Serial.print("Temperature");
      Serial.print("\t");
      Serial.print (cTemp);
      Serial.print(" C");

      Serial.print("\t");
      Serial.print("Pressure");
      Serial.print("\t");
      Serial.print(pressure,3);
      Serial.print(" mbar");

      Serial.print("\t");
      Serial.print("relativeAltitude");
      Serial.print("\t");
      Serial.print(relativeAltitude,0);
      Serial.println(" cm");
      
    #endif
    //convert data
    
    current_time = micros();
    time_difference = (current_time - previous_time)/1000000.0;

    // sensor fusion with complementary filter gyro and accelerator sensor
    #ifdef sensor_fusion 
      roll_gyro = 0.98 * roll_gyro + 0.02 * rollAngle;
      pitch_gyro = 0.98 * pitch_gyro + 0.02 * pitchAngle;
    #endif

    LoopTimer = micros();
    previous_time = current_time;
  }

  #ifdef debug1
    if (micros() - LoopTimer3 > 400000){
    Serial.print("Rate Roll");
    Serial.print("\t");
    Serial.print("");
    Serial.print("\t");
    Serial.println("Pitch");


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
