#ifndef CRSF_RECEIVER_H
#define CRSF_RECEIVER_H

#include <Arduino.h>
#include <array>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_POLY 0xD5
#define CRSF_MAX_CHANNELS 16
#define CRSF_MAX_FRAME_SIZE 64
#define CRSF_RC_PAYLOAD_SIZE 22  // 16 channels packed in 22 bytes

class CRSFReceiver {
public:
    CRSFReceiver(HardwareSerial &port);
    void begin();
    void update();
    bool newFrameAvailable() const;

    // Get raw 11-bit channel value (0–2047)
    uint16_t getChannelRaw(uint8_t ch) const;

    // Get all raw channels as std::array
    std::array<uint16_t, CRSF_MAX_CHANNELS> getChannelsRaw() const;

    // Optional: scale channel to 1000–2000 for flight controller
    uint16_t getChannelScaled(uint8_t ch) const;
    std::array<uint16_t, CRSF_MAX_CHANNELS> getChannelsScaled() const;

private:
    HardwareSerial &serial;
    uint8_t buffer[CRSF_MAX_FRAME_SIZE];
    uint8_t bufferIndex = 0;

    std::array<uint16_t, CRSF_MAX_CHANNELS> channelsRaw{};
    bool frameAvailable = false;

    uint8_t crc8_dvb_s2(const uint8_t *data, uint8_t len);
    void parseChannels(const uint8_t *payload);
};

#endif