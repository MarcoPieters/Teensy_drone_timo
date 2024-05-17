#include <Arduino.h>

// Define the serial port for iBUS communication
#define IBUS_SERIAL Serial2

// Buffer to store incoming iBUS data
uint8_t ibusBuffer[30];
uint32_t LoopTimer;
uint32_t LoopTimer2;

// Function to calculate checksum
uint16_t calculateChecksum(uint8_t *buffer, uint8_t length) {
  uint16_t checksum = 0xFFFF - 0x20 - 0x40;
  for (uint8_t i = 0; i < length; i++) {
    checksum -= buffer[i];
  }
  return checksum;
}

void setup() {
  Serial.begin(115200); // Initialize serial monitor for debugging
  IBUS_SERIAL.begin(115200); // Initialize iBUS serial communication
  LoopTimer = micros();
  LoopTimer2 = micros();
}

void loop() {
  // Look for the start bytes
  if (IBUS_SERIAL.available() > 1) {
    if (IBUS_SERIAL.read() == 0x20 && IBUS_SERIAL.read() == 0x40) {
      // Wait until we have enough data (32 bytes payload + 2 bytes checksum)
      while (IBUS_SERIAL.available() < 30);

      // Read the packet into the buffer
      IBUS_SERIAL.readBytes(ibusBuffer, 30);

      
      uint16_t receivedChecksum = ibusBuffer[28] | (ibusBuffer[29] << 8);

      // Calculate the checksum of the received packet
      uint16_t calculatedChecksum = calculateChecksum(ibusBuffer, 28);

      if (micros() - LoopTimer > 200000) {

        // Print raw data for debugging
        Serial.print("Raw Data: ");
        for (int i = 0; i < 30; i++) {
          Serial.print(ibusBuffer[i], HEX);
          Serial.print(" ");
          }
        Serial.print("Checksum: ");
        Serial.print(receivedChecksum, HEX);
        Serial.print(" Calculated: ");
        Serial.println(calculatedChecksum, HEX);
        LoopTimer = micros();
      }  

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
        NAN; //Serial.println("Checksum error");
      }
    } else {
      // If the start byte is not found, discard the byte and continue
      IBUS_SERIAL.read();
    }
  }

  // Delay to read every 7 milliseconds
  // delay(7);

  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }  
}
