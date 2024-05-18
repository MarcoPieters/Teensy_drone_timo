#include <Arduino.h>

// Define the serial port for iBUS communication
#define IBUS_SERIAL Serial2

// Buffer to store incoming iBUS data
uint8_t ibusBuffer[30];
uint8_t ibusWBuffer[32] = {0x20,0x40,0xdc,0x05,0xd0,0x07,0xe8,0x03,0xd0,0x04,0xd0,0x05,0xd0,0x06,0xd0,0x07,0xd0,0x08,0xd0,0x09,0xd0,0x0a,0xd0,0x0b,0xd0,0x0c,0xd0,0x0d,0xd0,0x0e,0xa9,0xf3};
uint32_t LoopTimer;
uint32_t LoopTimer2;

// Function to calculate checksum
uint16_t calculateChecksum(uint8_t *buffer, uint8_t length, uint8_t startByte1, uint8_t startByte2) {
  uint16_t checksum = 0xFFFF;
  checksum -= startByte1;
  checksum -= startByte2;
  for (uint8_t i = 0; i < length; i++) {
    checksum -= buffer[i];
  }
  return checksum;
}

void setup() {
  Serial.begin(115200); // Initialize serial monitor for debugging
  IBUS_SERIAL.begin(115200); // Initialize iBUS serial communication
  Serial7.begin(115200); // Initialize iBUS serial communication
  pinMode(LED_BUILTIN, OUTPUT); // Initialize LED pin
  LoopTimer = micros();
  LoopTimer2 = micros();
}

void loop() {
  static uint8_t startByte1 = 0;
  static uint8_t startByte2 = 0;

  // Non-blocking delay for every 7 milliseconds
  static uint32_t lastReadTime = 0;
  if (micros() - lastReadTime >= 7000) {
    lastReadTime = micros();
    // Your code to execute every 7 milliseconds
    for (int i = 0; i < 32; i++) {
        Serial7.write(ibusWBuffer[i]);
       }
  }
 
   // Look for the start bytes
  if (IBUS_SERIAL.available() >= 2) {
    startByte1 = IBUS_SERIAL.read();
    startByte2 = IBUS_SERIAL.read();

    if (startByte1 == 0x20 && startByte2 == 0x40) {
      // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
      while (IBUS_SERIAL.available() < 30);

      // Read the packet into the buffer
      IBUS_SERIAL.readBytes(ibusBuffer, 30);

      uint16_t receivedChecksum = ibusBuffer[28] | (ibusBuffer[29] << 8);

      // Calculate the checksum of the received packet including start bytes
      uint16_t calculatedChecksum = calculateChecksum(ibusBuffer, 28, startByte1, startByte2);

      if (micros() - LoopTimer > 200000) {
        // Print raw data for debugging
        Serial.print("Raw Data: ");
        Serial.print(startByte1, HEX);
        Serial.print(" ");
        Serial.print(startByte2, HEX);
        Serial.print(" ");
        for (int i = 0; i < 30; i++) {
          Serial.print(ibusBuffer[i], HEX);
          Serial.print(" ");
        }
        Serial.print("Checksum: ");
        Serial.print(receivedChecksum, HEX);
        Serial.print(" Calculated: ");
        Serial.println(calculatedChecksum, HEX);
        LoopTimer = micros();
      

      if (receivedChecksum == calculatedChecksum) {
        // Process the iBUS data if checksum is correct
        for (int i = 0; i < 14; i++) {
          uint16_t channelValue = ibusBuffer[2 * i + 1] << 8 | ibusBuffer[2 * i];
          Serial.print("Channel ");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.println(channelValue);
        }
      } else {
        Serial.println("Checksum error");
      }
      }
    } else {
      // If the start bytes are not found, discard the bytes and continue
      startByte1 = 0;
      startByte2 = 0;
    }
  }



  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }
}
