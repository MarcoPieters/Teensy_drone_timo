#include "CRSFReceiver.h"

CRSFReceiver::CRSFReceiver(HardwareSerial &port) : serial(port) {}

void CRSFReceiver::begin() {
    serial.begin(420000, SERIAL_8N1);
}

uint8_t CRSFReceiver::crc8_dvb_s2(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRSF_POLY) : (uint8_t)(crc << 1);
    }
    return crc;
}

void CRSFReceiver::parseChannels(const uint8_t *payload) {
    uint32_t bitBuffer = 0;
    uint8_t bitCount = 0;
    uint8_t payloadIndex = 0;

    for (uint8_t ch = 0; ch < CRSF_MAX_CHANNELS; ch++) {
        while (bitCount < 11) {
            bitBuffer |= ((uint32_t)payload[payloadIndex++]) << bitCount;
            bitCount += 8;
        }

        channelsRaw[ch] = bitBuffer & 0x07FF; // store raw 11-bit value
        bitBuffer >>= 11;
        bitCount -= 11;
    }

    frameAvailable = true;
}

void CRSFReceiver::update() {
    while (serial.available()) {
        uint8_t byteIn = serial.read();

        if (bufferIndex >= CRSF_MAX_FRAME_SIZE)
            bufferIndex = 0;

        buffer[bufferIndex++] = byteIn;

        if (bufferIndex < 2) continue;  // need at least address + length
        if (buffer[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER) { bufferIndex = 0; continue; }

        uint8_t length = buffer[1];
        if (length < 2 || length > CRSF_MAX_FRAME_SIZE - 2) { bufferIndex = 0; continue; }

        uint8_t totalLength = length + 2;
        if (bufferIndex < totalLength) continue;

        uint8_t frameType = buffer[2];
        const uint8_t *payload = &buffer[3];
        uint8_t receivedCRC = buffer[totalLength - 1];

        if (crc8_dvb_s2(&buffer[2], length - 1) == receivedCRC) {
            if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED &&
                (length - 2) == CRSF_RC_PAYLOAD_SIZE) {
                parseChannels(payload);
            }
        }

        bufferIndex = 0;  // reset buffer
    }
}

bool CRSFReceiver::newFrameAvailable() const {
    return frameAvailable;
}

uint16_t CRSFReceiver::getChannelRaw(uint8_t ch) const {
    return (ch < CRSF_MAX_CHANNELS) ? channelsRaw[ch] : 0;
}

std::array<uint16_t, CRSF_MAX_CHANNELS> CRSFReceiver::getChannelsRaw() const {
    return channelsRaw;
}

// Scale to 1000–2000 for FC
uint16_t CRSFReceiver::getChannelScaled(uint8_t ch) const {
    if (ch >= CRSF_MAX_CHANNELS) return 0;
    return 1000 + ((channelsRaw[ch] - 172) * 1000UL) / 1639;
}

std::array<uint16_t, CRSF_MAX_CHANNELS> CRSFReceiver::getChannelsScaled() const {
    std::array<uint16_t, CRSF_MAX_CHANNELS> scaled{};
    for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; i++)
        scaled[i] = getChannelScaled(i);
    return scaled;
}