#include "CRSFReceiver.h"

CRSFReceiver::CRSFReceiver(HardwareSerial &port)
    : serial(port) {}

void CRSFReceiver::begin() {
    serial.begin(420000, SERIAL_8N1);
}

uint8_t CRSFReceiver::crc8_dvb_s2(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = ((crc << 1) ^ CRSF_POLY) & 0xFF;
            else
                crc = (crc << 1) & 0xFF;
        }
    }
    return crc;
}

void CRSFReceiver::parseChannels(uint8_t *payload) {

    uint32_t bitBuffer = 0;
    uint8_t bitCount = 0;
    uint8_t payloadIndex = 0;

    for (uint8_t ch = 0; ch < 16; ch++) {

        while (bitCount < 11) {
            bitBuffer |= ((uint32_t)payload[payloadIndex++]) << bitCount;
            bitCount += 8;
        }

        channels[ch] = bitBuffer & 0x7FF;
        bitBuffer >>= 11;
        bitCount -= 11;
    }
}

void CRSFReceiver::update() {

    while (serial.available()) {

        if (bufferIndex >= CRSF_MAX_FRAME_SIZE)
            bufferIndex = 0;

        buffer[bufferIndex++] = serial.read();

        if (bufferIndex < 2)
            continue;

        if (buffer[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER) {
            memmove(buffer, buffer + 1, --bufferIndex);
            continue;
        }

        uint8_t length = buffer[1];

        if (length < 2 || length > CRSF_MAX_FRAME_SIZE) {
            memmove(buffer, buffer + 1, --bufferIndex);
            continue;
        }

        uint8_t totalLength = length + 2;

        if (bufferIndex < totalLength)
            continue;

        uint8_t frameType = buffer[2];
        uint8_t *payload = &buffer[3];
        uint8_t receivedCRC = buffer[totalLength - 1];

        if (crc8_dvb_s2(&buffer[2], length - 1) != receivedCRC) {
            bufferIndex = 0;
            continue;
        }

        if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            if ((length - 2) == 22) {
                parseChannels(payload);
                frameAvailable = true;
            }
        }

        bufferIndex = 0;
    }
}

bool CRSFReceiver::newFrameAvailable() {
    if (frameAvailable) {
        frameAvailable = false;
        return true;
    }
    return false;
}

uint16_t CRSFReceiver::getChannel(uint8_t ch) {
    if (ch < 16)
        return channels[ch];
    return 0;
}