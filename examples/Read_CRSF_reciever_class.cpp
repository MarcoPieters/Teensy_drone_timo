#include <Arduino.h>

// #define SBUS_READ     // SBUS (130Hz / 7.7ms)
#define CRSF_READ  // CRSF (250Hz / 4ms)

#ifdef SBUS_READ
  #include "IBusReceiver.h"
  IBusReceiver ibus(Serial7);
  #define RECEIVER_INTERVAL 7700
#endif

#ifdef CRSF_READ
  #include "CRSFReceiver.h"
  CRSFReceiver crsf(Serial6);
  #define RECEIVER_INTERVAL 4000
#endif


// Timer variables
uint32_t ReceiverTimer;
uint32_t StatsTimer;

// Statistics
unsigned long successfulReads = 0;
unsigned long missedReads = 0;

void setup() {

#ifdef SBUS_READ
    ibus.begin(115200);
#endif

#ifdef CRSF_READ
    crsf.begin();
#endif

    Serial.begin(115200);
    Serial.println("Start");

    ReceiverTimer = micros();
    StatsTimer = millis();
}

void loop() {

#ifdef SBUS_READ
    if (micros() - ReceiverTimer >= RECEIVER_INTERVAL) {
        ReceiverTimer += RECEIVER_INTERVAL;

        if (ibus.readChannels()) {
            successfulReads++;

            for (int i = 1; i <= IBUS_CHANNELS; i++) {
                Serial.print("Ch");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(ibus.getChannelValue(i));
                Serial.print(" ");
            }
            Serial.println();
        }
        else {
            missedReads++;
        }
    }
#endif


#ifdef CRSF_READ
    // Parse serial continuously
    crsf.update();

    if (micros() - ReceiverTimer >= RECEIVER_INTERVAL) {
        ReceiverTimer += RECEIVER_INTERVAL;

        if (crsf.newFrameAvailable()) {
            successfulReads++;

            for (int i = 0; i < 16; i++) {
                Serial.print("Ch");
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.print(crsf.getChannel(i));
                Serial.print(" ");
            }
            Serial.println();
        }
        else {
            missedReads++;
        }
    }
#endif


    // Print statistics every 770ms
    if (millis() - StatsTimer > 770) {
        StatsTimer = millis();

        Serial.print("Successful reads: ");
        Serial.print(successfulReads);
        Serial.print(" Missed reads: ");
        Serial.println(missedReads);

        successfulReads = 0;
        missedReads = 0;
    }
}