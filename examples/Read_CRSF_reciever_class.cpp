#include <Arduino.h>

// #define SBUS_READ     // SBUS (130Hz / 7.7ms)
#define CRSF_READ  // CRSF (250Hz / 4ms)
#define power_stick_threshold_on
#define MAX_CHANNELS 12
int ReceiverValue[MAX_CHANNELS] = {0}; // Array to hold receiver channel values  

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

#ifdef SBUS_READ
    // Function to read IBUS signals from RC receiver via RX UART pin.
    void read_receiver(void) {
    if (ibus.readChannels()) {
        for (int i = 1; i <= IBUS_CHANNELS; i++) {
            ReceiverValue[i-1] = int(ibus.getChannelValue(i));
        }
    }
    }  
#endif

#ifdef CRSF_READ
// Function to read CRSF signals from RC receiver via RX UART pin.
void read_receiver(void) {
  if (crsf.newFrameAvailable()) {
        // Get scaled values 1000–2000
        for (int i = 0; i <= MAX_CHANNELS-1; i++) {
        ReceiverValue[i] = int(crsf.getChannelScaled(i));
      }
  }
}  
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

    ReceiverTimer = millis();
    StatsTimer = millis();

#ifdef power_stick_threshold_on
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1200) {
    #ifdef CRSF_READ
    crsf.update();
    #endif
    read_receiver();
    Serial.print("waiting for throttle stick to be in the middle position...");
    Serial.println(ReceiverValue[2]);
    delay(4);
  }
  #endif

}

void loop() {

#ifdef SBUS_READ
    if (millis() - ReceiverTimer >= RECEIVER_INTERVAL / 1000) {
        ReceiverTimer = millis();

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

    read_receiver();

#endif

    // Print statistics every 770ms
    if (millis() - StatsTimer > 10) {
        StatsTimer = millis();
        for (int i = 0; i < MAX_CHANNELS; i++) {
            Serial.print("Ch"); 
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(ReceiverValue[i]);
            Serial.print(" ");
        }
        Serial.println();
        // Serial.print("Successful reads: ");
        // Serial.print(successfulReads);
        // Serial.print(" Missed reads: ");
        // Serial.println(missedReads);

        successfulReads = 0;
        missedReads = 0;
    }
}