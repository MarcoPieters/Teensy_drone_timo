#include <Arduino.h>

// Define the serial port for iBUS communication
#define IBUS_SERIAL Serial2

#define DEBUG
//#define TEST_SIGNAL

// Buffer to store incoming iBUS data
uint8_t ibusBuffer[32];
uint8_t ibusWBuffer[28] = {
  0xdc, 0x05, 0xd0, 0x07, 0xe8, 0x03, 0xd0, 0x07, 0xdc, 0x05, 0xd0, 0x06, 
  0xd0, 0x07, 0xd0, 0x08, 0xd0, 0x09, 0xd0, 0x0a, 0xd0, 0x0b, 0xd0, 0x0c, 
  0xd0, 0x0d, 0xd0, 0x0e
}; // packet: 2 bytes header + 2 * 14 bytes data channels + 2 bytes checksum

uint32_t LoopTimer;
uint32_t LoopTimer1;
uint32_t LoopTimer2;
uint32_t last = millis();

// Function to calculate checksum
uint16_t calculateChecksum(uint8_t protocol_length, uint8_t protocol_command_adr, uint8_t *buffer, uint8_t length) {
  uint16_t checksum = 0xFFFF;
  checksum -= protocol_length;
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
      Serial.println("Timeout");
      return false; // Timeout occurred
    }
  }
  IBUS_SERIAL.readBytes(buffer, length);
  return true; // Successfully read the packet
}

void sendIBUSWriteBuffer(uint8_t protocol_length, uint8_t protocol_command_adr, uint8_t *buffer, size_t length) {
  uint16_t checksum = calculateChecksum(protocol_length, protocol_command_adr, buffer, length);
  uint8_t checksumLow = checksum & 0xFF;
  uint8_t checksumHigh = (checksum >> 8) & 0xFF;

  Serial7.write(protocol_length);
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
  static uint8_t protocol_length = 0;
  static uint8_t protocol_command_adr = 0;
  static uint8_t protocol_command = 0;
  static uint8_t protocol_address = 0;

  #ifdef TEST_SIGNAL
    // Non-blocking delay for every 7 milliseconds
    if (micros() - LoopTimer1 > 7000) { 
      sendIBUSWriteBuffer(0x20, 0x41, ibusWBuffer, 28);
      LoopTimer1 = micros();
    }
  #endif
  // Look for the start bytes
  if (IBUS_SERIAL.available() > 1) {
    // only consider a new data package if we have not heard anything for >3ms
    uint32_t now = millis();
    if (now - last >= 3) {
      last = now;
      if (readIBUSPacket(ibusBuffer, 2, 10)) { // 10 millisecond timeout
        protocol_length = ibusBuffer[0];
        protocol_command_adr = ibusBuffer[1]; // bit 4..7 command; bit 0..3 address
        protocol_command = protocol_command_adr & 0xF0; // 0x40 servomode; 0x80 discover sensors; 0x90 request sensortype; 0xA0 request measurement
        protocol_address = protocol_command_adr & 0x0F;

        #ifdef DEBUG
          Serial.print("length ");
          Serial.print(protocol_length, HEX);
          Serial.print(" ");
          Serial.print(protocol_command_adr, HEX);
          Serial.print(" command ");
          Serial.print(protocol_command, HEX);
          Serial.print(" address ");
          Serial.print(protocol_address, HEX);
          Serial.print(" | ");
        #endif

        if (protocol_length == 0x20 && protocol_command == 0x40) {
          // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
          if (readIBUSPacket(ibusBuffer, protocol_length - 0x02, 10)) { // 10 millisecond timeout
            uint16_t receivedChecksum = ibusBuffer[protocol_length - 4] | (ibusBuffer[protocol_length - 3] << 8);

            // Calculate the checksum of the received packet including start bytes
            uint16_t calculatedChecksum = calculateChecksum(protocol_length, protocol_command_adr, ibusBuffer, protocol_length - 0x04);
          #ifdef DEBUG
            if (micros() - LoopTimer > 200) {
              // Print raw data for debugging
              Serial.println();
              Serial.print("Raw Data: ");
              Serial.print(protocol_length, HEX);
              Serial.print(" ");
              Serial.print(protocol_command_adr, HEX);
              Serial.print(" ");
              for (int i = 0; i < protocol_length - 0x02; i++) {
                Serial.print(ibusBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.print("Checksum: ");
              Serial.print(receivedChecksum, HEX);
              Serial.print(" Calculated: ");
              Serial.println(calculatedChecksum, HEX);
              LoopTimer = micros();
              
            }
            #endif
            
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
          } else {
            // Timeout occurred or not enough data
            Serial.println("Packet read timeout or incomplete packet");
          }
        } else if (protocol_length == 0x04 && protocol_command == 0x80) {
          // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
          if (readIBUSPacket(ibusBuffer, protocol_length - 0x02, 10)) { // 10 millisecond timeout
            uint16_t receivedChecksum = ibusBuffer[protocol_length - 4] | (ibusBuffer[protocol_length - 3] << 8);

            // Calculate the checksum of the received packet including start bytes
            uint16_t calculatedChecksum = calculateChecksum(protocol_length, protocol_command_adr, ibusBuffer, protocol_length - 0x04);
          Serial.print("Discover sensor ");
            #ifdef DEBUG
            // Print raw data for debugging
              Serial.print("Raw Data: ");
              Serial.print(protocol_length, HEX);
              Serial.print(" ");
              Serial.print(protocol_command_adr, HEX);
              Serial.print(" ");
              for (int i = 0; i < protocol_length - 0x02; i++) {
                Serial.print(ibusBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.print("Checksum: ");
              Serial.print(receivedChecksum, HEX);
              Serial.print(" Calculated: ");
              Serial.println(calculatedChecksum, HEX);
              #endif

          Serial.println();
          //delayMicroseconds(200);
          sendIBUSWriteBuffer(protocol_length, 0x81, {}, 0); // Empty buffer for response
          
        }} else if (protocol_length == 0x04 && protocol_command == 0x90) {
          // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
          if (readIBUSPacket(ibusBuffer, protocol_length - 0x02, 10)) { // 10 millisecond timeout
            uint16_t receivedChecksum = ibusBuffer[protocol_length - 4] | (ibusBuffer[protocol_length - 3] << 8);

            // Calculate the checksum of the received packet including start bytes
            uint16_t calculatedChecksum = calculateChecksum(protocol_length, protocol_command_adr, ibusBuffer, protocol_length - 0x04);
          Serial.print("Request sensor type ");
            #ifdef DEBUG
            // Print raw data for debugging
              Serial.print("Raw Data: ");
              Serial.print(protocol_length, HEX);
              Serial.print(" ");
              Serial.print(protocol_command_adr, HEX);
              Serial.print(" ");
              for (int i = 0; i < protocol_length - 0x02; i++) {
                Serial.print(ibusBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.print("Checksum: ");
              Serial.print(receivedChecksum, HEX);
              Serial.print(" Calculated: ");
              Serial.println(calculatedChecksum, HEX);
              #endif

          Serial.println();
          uint8_t buffer[2] = {0x00,0x02}; // volt = 0x00 : temperature = 0x01 : Mot rpm = 0x02 : volt = 0x03 
          //delayMicroseconds(200);
          sendIBUSWriteBuffer(0x06, 0x91, buffer, 2); // Empty buffer for response
        }} else if (protocol_length == 0x04 && protocol_command == 0xA0) {
          // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
          if (readIBUSPacket(ibusBuffer, protocol_length - 0x02, 10)) { // 10 millisecond timeout
            uint16_t receivedChecksum = ibusBuffer[protocol_length - 4] | (ibusBuffer[protocol_length - 3] << 8);

            // Calculate the checksum of the received packet including start bytes
            uint16_t calculatedChecksum = calculateChecksum(protocol_length, protocol_command_adr, ibusBuffer, protocol_length - 0x04);
          Serial.print("Request measurement ");
            #ifdef DEBUG
            // Print raw data for debugging
              Serial.print("Raw Data: ");
              Serial.print(protocol_length, HEX);
              Serial.print(" ");
              Serial.print(protocol_command_adr, HEX);
              Serial.print(" ");
              for (int i = 0; i < protocol_length - 0x02; i++) {
                Serial.print(ibusBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.print("Checksum: ");
              Serial.print(receivedChecksum, HEX);
              Serial.print(" Calculated: ");
              Serial.println(calculatedChecksum, HEX);
              #endif

          Serial.println();
          uint8_t buffer[2] = {0x00,0x80};
          //delayMicroseconds(200);
          sendIBUSWriteBuffer(0x06, 0xA1, buffer, 2); // Empty buffer for response
        }} else if (protocol_length == 0x06 && protocol_command == 0xA0) {
          // Wait until we have enough data (28 bytes payload + 2 bytes checksum)
          if (readIBUSPacket(ibusBuffer, protocol_length - 0x02, 10)) { // 10 millisecond timeout
            uint16_t receivedChecksum = ibusBuffer[protocol_length - 4] | (ibusBuffer[protocol_length - 3] << 8);

            // Calculate the checksum of the received packet including start bytes
            uint16_t calculatedChecksum = calculateChecksum(protocol_length, protocol_command_adr, ibusBuffer, protocol_length - 0x04);
          Serial.print("Recieved measurement ");
            #ifdef DEBUG
            // Print raw data for debugging
              Serial.print("Raw Data: ");
              Serial.print(protocol_length, HEX);
              Serial.print(" ");
              Serial.print(protocol_command_adr, HEX);
              Serial.print(" ");
              for (int i = 0; i < protocol_length - 0x02; i++) {
                Serial.print(ibusBuffer[i], HEX);
                Serial.print(" ");
              }
              Serial.print("Checksum: ");
              Serial.print(receivedChecksum, HEX);
              Serial.print(" Calculated: ");
              Serial.println(calculatedChecksum, HEX);
              #endif

          Serial.println();
          //uint8_t buffer[2] = {0xB0,0x02};
          //delayMicroseconds(200);
          //sendIBUSWriteBuffer(0x06, 0xA1, buffer, 2); // Empty buffer for response
        }} else {
          // If the start bytes are not found, discard the bytes and continue
          protocol_length = 0;
          protocol_command_adr = 0;
          while (IBUS_SERIAL.available()) {
            IBUS_SERIAL.read(); 
            Serial.print("flush");
          }
          Serial.println("No correct Header found in Packet");
        }
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