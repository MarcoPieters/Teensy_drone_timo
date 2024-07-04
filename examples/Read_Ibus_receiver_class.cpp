#include <Arduino.h>
#include "IBusReceiver.h"

// Create an IBusReceiver object for Serial7 (change Serial7 to Serial1 or appropriate Serial if needed)
IBusReceiver ibus(Serial7);

// Timer variables for main loop and statistics
uint32_t LoopTimer;
uint32_t StatsTimer;
bool newDataAvailable = false;

// Counters for statistics
unsigned long successfulReads = 0;
unsigned long missedReads = 0;

void setup() {
    ibus.begin(115200);  // Initialize iBus communication at 115200 baud rate
    Serial.begin(115200); // For debugging
    Serial.println("Start");
    // Start loop timer
    LoopTimer = micros();
    StatsTimer = millis(); // Start statistics timer
}

void loop() {
    // Check for new data every 7700us
    if (micros() - LoopTimer > 7700) {
        LoopTimer = micros();
        newDataAvailable = ibus.readChannels();
        if (newDataAvailable) {
            successfulReads++;
            for (int i = 1; i <= IBUS_CHANNELS; i++) {
                Serial.print("Ch");
                Serial.print(i);
                Serial.print(": ");
                Serial.print(ibus.getChannelValue(i));
                Serial.print(" ");
            }
          Serial.println();  
        } else {
            missedReads++;
            // Serial.println("No new data available"); // Optional: Uncomment for debugging
        }
    }

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
