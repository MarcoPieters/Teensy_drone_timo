#include <Arduino.h>

// Define the serial port for iBUS communication
#define IBUS_SERIAL Serial2

#define DEBUG

// Buffer to store incoming iBUS data
uint8_t ibusBuffer[32];
uint8_t ibusWBuffer[28] = {0xdc, 0x05, 0xd0, 0x07, 0xe8, 0x03, 0xd0, 0x07, 0xdc, 0x05, 0xd0, 0x06, 0xd0, 0x07, 0xd0, 0x08, 0xd0, 0x09, 0xd0, 0x0a, 0xd0, 0x0b, 0xd0, 0x0c, 0xd0, 0x0d, 0xd0, 0x0e}; // packet: 2 bytes header + 2 * 14 bytes data channels + 2 bytes checksum
uint32_t LoopTimer;
uint32_t LoopTimer1;
uint32_t LoopTimer2;

// Function to calculate checksum
uint16_t calculateChecksum(uint8_t protocol_lenght, uint8_t protocol_command_adr,uint8_t *buffer, uint8_t length) {
  uint16_t checksum = 0xFFFF;
  checksum -= protocol_lenght;
  checksum -= protocol_command_adr;
  for (uint8_t i = 0; i < length; i++) {
    checksum -= buffer[i];
  }
  return checksum;
}

bool readIBUSPacket(uint8_t *buffer, int length, uint32_t timeout) {
  uint32_t startTime = millis();
  while (IBUS_SERIAL.available() < length) {
    if (millis() - startTime >= timeout) {
      Serial.print("Time out");
      return false; // Timeout occurred
      
    }
  }
  IBUS_SERIAL.readBytes(buffer, length);
  return true; // Successfully read the packet
}

void sendIBUSWriteBuffer(uint8_t protocol_lenght, uint8_t protocol_command_adr, uint8_t *buffer, size_t length) {
  uint16_t checksum = calculateChecksum(protocol_lenght, protocol_command_adr, buffer, length);
  uint8_t checksumLow = checksum & 0xFF;
  uint8_t checksumHigh = (checksum >> 8) & 0xFF;

  Serial7.write(protocol_lenght);
  Serial7.write(protocol_command_adr);
  for (size_t i = 0; i < length; i++) {
    Serial7.write(buffer[i]);
  }
  Serial7.write(checksumLow);
  Serial7.write(checksumHigh);
}

void setup() {
  Serial.begin(115200); // Initialize serial monitor for debugging
  IBUS_SERIAL.begin(115200); // Initialize iBUS serial communication
  Serial7.begin(115200); // Initialize secondary serial communication
  pinMode(LED_BUILTIN, OUTPUT); // Initialize LED pin
  LoopTimer = micros();
  LoopTimer1 = micros();
  LoopTimer2 = micros();
}

void loop() {
  static uint8_t protocol_lenght = 0;
  static uint8_t protocol_command_adr = 0;
  static uint8_t protocol_command = 0;
  static uint8_t protocol_address = 0;

  // Non-blocking delay for every 7 milliseconds
  
  if (micros() - LoopTimer1 > 7000) { 
    sendIBUSWriteBuffer(0x20, 0x41, ibusWBuffer, 28);
    LoopTimer1 = micros();
    }

  // Look for the start bytes
  if (IBUS_SERIAL.available()>0){
    if (readIBUSPacket(ibusBuffer, 2, 10)) { // 10 milisecond timeout
      protocol_lenght = ibusBuffer[0] ;
      protocol_command_adr = ibusBuffer[1]; // bit 4..7 command ; bit 0..3 address
      protocol_command = protocol_command_adr & 0xF0; //0x40 servomode; 0x80 discover sensors 0x90 request sensortype 0xA0 request measurement
      protocol_address = protocol_address & 0x0F;

      if (protocol_lenght == 0x20 && protocol_command == 0x40) {
        // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
        if (readIBUSPacket(ibusBuffer, protocol_lenght - 0x02, 10)) { // 10 milisecond timeout

          uint16_t receivedChecksum = ibusBuffer[28] | (ibusBuffer[29] << 8);

          // Calculate the checksum of the received packet including start bytes
          uint16_t calculatedChecksum = calculateChecksum(protocol_lenght, protocol_command_adr,ibusBuffer, protocol_lenght - 0x04);

          if (micros() - LoopTimer > 200000) {
            // Print raw data for debugging
            Serial.print("Raw Data: ");
            Serial.print(protocol_lenght, HEX);
            Serial.print(" ");
            Serial.print(protocol_command_adr, HEX);
            Serial.print(" ");
            for (int i = 0; i < protocol_lenght - 0x02; i++) {
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
          }}
        } else {
          // Timeout occurred or not enough data
          Serial.println("Packet read timeout or incomplete packet");
        }
      } else {
        // If the start bytes are not found, discard the bytes and continue
        protocol_lenght = 0;
        protocol_command_adr = 0;
        Serial.println("No correct Header found in Packet");
      }
    }
  }

  if (micros() - LoopTimer2 > 200000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }
}
