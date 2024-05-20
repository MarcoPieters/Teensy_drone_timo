#include <Arduino.h>
/*
IBUS protocol :  header 2 bytes + 28 bytes data 14 channels + checksum 2 bytes
Header protocollenght + commmandcode  0x20 + 0x40
Data channels : 2 bytes for each channel litte endian LSB + MSB; example: 1500 = 0xDC + 0x05; 1000 = 0xE8 + 0x03; 2000 = 0xD0 + 0x08
CRC 30 bytes including headerbytes and databytes. starting with 0xFFFF minus each byte. Leftover is checksum value. 
*/


#define DEBUG

// Buffer to store incoming iBUS data
uint8_t ibusBuffer[30];
uint8_t ibusWBuffer[6] = {0x06, 0xa1, 0x34, 0x12}; // packet:
uint32_t LoopTimer;
uint32_t LoopTimer2;

// Function to calculate checksum
uint16_t calculateChecksum(uint8_t *buffer, uint8_t length) {
  uint16_t checksum = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    checksum -= buffer[i];
  }
  return checksum;
}

void sendIBUSWriteBuffer(uint8_t *buffer, size_t length) {
  uint16_t checksum = calculateChecksum(buffer, length);
  uint8_t checksumLow = checksum & 0xFF;
  uint8_t checksumHigh = (checksum >> 8) & 0xFF;

  for (size_t i = 0; i < length; i++) {
    Serial7.write(buffer[i]);
  }
}

void setup() {
  Serial.begin(115200); // Initialize serial monitor for debugging
  Serial7.begin(115200); // Initialize secondary serial communication
  pinMode(LED_BUILTIN, OUTPUT); // Initialize LED pin
  LoopTimer = micros();
  LoopTimer2 = micros();
}

void loop() {

  // Non-blocking delay for every 7 milliseconds
  static uint32_t lastReadTime = 0;
  if (micros() - lastReadTime >= 7000) {
    lastReadTime = micros();
    sendIBUSWriteBuffer(ibusWBuffer, 4);
    }

  
  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }
}
