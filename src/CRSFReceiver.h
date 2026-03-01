#ifndef CRSF_RECEIVER_H
#define CRSF_RECEIVER_H

#include <Arduino.h>

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_POLY 0xD5
#define CRSF_MAX_CHANNELS 16
#define CRSF_MAX_FRAME_SIZE 64

class CRSFReceiver {
public:
    CRSFReceiver(HardwareSerial &port);
    void begin();
    void update();
    bool newFrameAvailable();
    uint16_t getChannel(uint8_t ch);

private:
    HardwareSerial &serial;

    uint8_t buffer[CRSF_MAX_FRAME_SIZE];
    uint8_t bufferIndex = 0;

    uint16_t channels[CRSF_MAX_CHANNELS];
    bool frameAvailable = false;

    uint8_t crc8_dvb_s2(uint8_t *data, uint8_t len);
    void parseChannels(uint8_t *payload);
};

#endif