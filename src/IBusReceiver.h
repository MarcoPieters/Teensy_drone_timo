#ifndef IBUSRECEIVER_H
#define IBUSRECEIVER_H

#include <Arduino.h>

// Constants for iBus
#define IBUS_BUFFSIZE 32
#define IBUS_CHANNELS 10
#define IBUS_HEADER 0x20

class IBusReceiver {
public:
    IBusReceiver(HardwareSerial &serialPort);
    void begin(long baudRate);
    bool readChannels();
    int getChannelValue(int channel);

private:
    HardwareSerial &_serial;
    uint8_t buffer[IBUS_BUFFSIZE];
    uint16_t channels[IBUS_CHANNELS];
    int bufferIndex;
    uint32_t lastReadTime; // To track time between packets

    bool validateChecksum();
    bool syncWithIBus();
};

#endif // IBUSRECEIVER_H
