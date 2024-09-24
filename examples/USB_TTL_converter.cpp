#include <Arduino.h>

static const uint32_t Baud = 38400;
static const uint32_t Baud1 = 38400;

void setup() {
  Serial.begin(Baud); // Uncomment if you need Serial for debugging
  Serial5.begin(Baud1);
}

void loop() {
  while (Serial5.available() > 0) {
    Serial.write(Serial5.read());
  }

  while (Serial.available() > 0) {
    Serial5.write(Serial.read());
  }

}
