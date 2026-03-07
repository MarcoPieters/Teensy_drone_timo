#include <Arduino.h>

// #define SBUS_READ     // SBUS (130Hz / 7.7ms)
#define CRSF_READ  // CRSF (250Hz / 4ms)
#define power_stick_threshold_on

#ifdef SBUS_READ
  #include "IBusReceiver.h"
  IBusReceiver ibus(Serial7);
  #define RECEIVER_INTERVAL 7700
   #define MAX_CHANNELS IBUS_CHANNELS // IBUS supports up to 12 channels, but we will read only 12 for this exampl
#endif

#ifdef CRSF_READ
  #include "CRSFReceiver.h"
  CRSFReceiver crsf(Serial6);
  #define RECEIVER_INTERVAL 4000
  #define MAX_CHANNELS CRSF_MAX_CHANNELS // CRSF supports up to 16 channels, but we will read only 12 for this example
#endif

int ReceiverValue[MAX_CHANNELS] = {0}; // Array to hold receiver channel values  

#ifdef SBUS_READ
    // Function to read IBUS signals from RC receiver via RX UART pin.
    void read_receiver(void) {
    if (ibus.readChannels()) {
        for (int i = 1; i <= MAX_CHANNELS; i++) {
            ReceiverValue[i-1] = int(ibus.getChannelValue(i));
        }
    }
    }  
#endif

#ifdef CRSF_READ
// Function to read CRSF signals from RC receiver via RX UART pin.
void read_receiver(void) {
  crsf.update();
  if (crsf.newFrameAvailable()) {
        // Get scaled values 1000–2000
        for (int i = 0; i <= MAX_CHANNELS-1; i++) {
        ReceiverValue[i] = int(crsf.getChannelScaled(i));
      }
  crsf.frameAvailable = false;  // Reset frame flag after reading
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
    read_receiver();
    Serial.print("waiting for throttle stick to be in the middle position...");
    Serial.println(ReceiverValue[2]);
  }
  #endif

}

void loop() {
// start timing
uint32_t t0 = micros();

#ifdef SBUS_READ
    if (millis() - ReceiverTimer >= RECEIVER_INTERVAL / 1000) {
        ReceiverTimer = millis();
        read_receiver();
    }
#endif

#ifdef CRSF_READ
  read_receiver();
#endif

// compute loop time in microseconds
uint32_t dt = micros() - t0;

    // Print statistics every 770ms
    if (millis() - StatsTimer > 5) {
        StatsTimer = millis();
        // Serial.print("loop "); Serial.print(dt); Serial.print(" us ");
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