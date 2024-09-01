#include "IBusReceiver.h"

IBusReceiver::IBusReceiver(HardwareSerial &serialPort) : _serial(serialPort), bufferIndex(0), lastReadTime(0) {}

void IBusReceiver::begin(long baudRate) {
    _serial.begin(baudRate);
}

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

bool IBusReceiver::readChannels() {
    // Check if there was a 3ms gap since the last read
    if (micros() - lastReadTime < 3000) {
        return false;
    }

    if (bufferIndex == 0) {
        if (!syncWithIBus()) {
            return false;
        }
    }

    while (_serial.available() && bufferIndex < IBUS_BUFFSIZE) {
        buffer[bufferIndex++] = _serial.read();
    }

    if (bufferIndex == IBUS_BUFFSIZE) {
        if (validateChecksum()) {
            for (int i = 0; i < IBUS_CHANNELS; i++) {
                channels[i] = buffer[2 * i + 3] << 8 | buffer[2 * i + 2];
            }
            bufferIndex = 0; // Reset for the next packet
            lastReadTime = micros(); // Update the last read time
            return true;
        }
        bufferIndex = 0; // Reset for the next attempt
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
