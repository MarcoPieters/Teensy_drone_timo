#include "IBusReceiver.h"

// Constructor to initialize the IBusReceiver object
IBusReceiver::IBusReceiver(HardwareSerial &serialPort) : _serial(serialPort), bufferIndex(0), lastReadTime(0) {}

// Begin serial communication with the specified baud rate
void IBusReceiver::begin(long baudRate) {
    _serial.begin(baudRate);
}

// Synchronize with the IBus by finding the header byte
bool IBusReceiver::syncWithIBus() {
    while (_serial.available()) {
        uint8_t incomingByte = _serial.read();
        if (incomingByte == IBUS_HEADER) {
            bufferIndex = 0;
            buffer[bufferIndex++] = incomingByte;
            lastReadTime = micros();
            return true;
        }
    }
    return false;
}

// Read channel data from the IBus receiver
bool IBusReceiver::readChannels() {
    // Check if there was a 3ms gap since the last read
    if (micros() - lastReadTime < 3000) {
        return false;
    }

    // If bufferIndex is 0, try to sync with IBus
    if (bufferIndex == 0) {
        if (!syncWithIBus()) {
            return false;
        }
    }

    // Read bytes from the serial port into the buffer
    while (_serial.available() && bufferIndex < IBUS_BUFFSIZE) {
        buffer[bufferIndex++] = _serial.read();
    }

    // If the buffer is full, validate the checksum
    if (bufferIndex == IBUS_BUFFSIZE) {
        if (validateChecksum()) {
            // Extract channel data from the buffer
            for (int i = 0; i < IBUS_CHANNELS; i++) {
                channels[i] = buffer[2 * i + 3] << 8 | buffer[2 * i + 2];
            }
            bufferIndex = 0; // Reset for the next packet
            lastReadTime = micros(); // Update the last read time
            return true;
        }
        bufferIndex = 0; // Reset for the next attempt if checksum fails
    }
    return false;
}

int IBusReceiver::getChannelValue(int channel) {
    if (channel < 1 || channel > IBUS_CHANNELS) {
        return -1; // Invalid channel
    }
    return channels[channel - 1];
}

bool IBusReceiver::validateChecksum() {
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < IBUS_BUFFSIZE - 2; i++) {
        checksum -= buffer[i];
    }
    uint16_t receivedChecksum = buffer[IBUS_BUFFSIZE - 2] | (buffer[IBUS_BUFFSIZE - 1] << 8);
    return (checksum == receivedChecksum);
}
