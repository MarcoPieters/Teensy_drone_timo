#include <Arduino.h>

static const uint32_t Baud = 38400;
static const uint32_t Baud1 = 38400;

void setup() {
  Serial.begin(Baud); // Uncomment if you need Serial for debugging
  Serial2.begin(Baud1);
}

void loop() {
  while (Serial2.available() > 0) {
    Serial.write(Serial2.read());
  }

  while (Serial.available() > 0) {
    Serial2.write(Serial.read());
  }

}
