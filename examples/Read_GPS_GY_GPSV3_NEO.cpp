#include <TinyGPS++.h>

const uint32_t GPSBaud = 38400;
uint32_t LoopTimer2;

TinyGPSPlus gps;

bool checkGPSConnection() {
  // Check for data from GPS module
  for (int i = 0; i < 10; ++i) {
    if (Serial2.available()) {
      return true;
    }
    delay(500); // Wait 500ms before checking again
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GPSBaud);

  // Set pin modes and initial states
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  // Check GPS connection
  if (!checkGPSConnection()) {
    Serial.println("GPS module not detected. Please check the connection.");
    while (1); // halt the program
  } else {
    Serial.println("GPS module connected successfully.");
  }
  
  LoopTimer2 = micros();
}

void loop() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
      } else {
        Serial.println("Location: Not Available");
      }
      if (gps.altitude.isValid()) {
        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" meters");
      } else {
        Serial.println("Altitude: Not Available");
      }
      if (gps.speed.isValid()) {
        Serial.print("Speed: ");
        Serial.print(gps.speed.kmph());
        Serial.println(" km/h");
      } else {
        Serial.println("Speed: Not Available");
      }
      if (gps.satellites.isValid()) {
        Serial.print("Number of Satellites: ");
        Serial.println(gps.satellites.value());
      } else {
        Serial.println("Number of Satellites: Not Available");
      }
      if (gps.course.isValid()) {
        Serial.print("Course (heading): ");
        Serial.print(gps.course.deg());
        Serial.println(" degrees");
      } else {
        Serial.println("Course (heading): Not Available");
      }
      if (gps.date.isValid()) {
        Serial.print("Date UTC: ");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.println(gps.date.year());
      } else {
        Serial.println("Date: Not Available");
      }
      if (gps.time.isValid()) {
        Serial.print("Time UTC: ");
        if (gps.time.hour() < 10) Serial.print("0");
        Serial.print(gps.time.hour());
        Serial.print(":");
        if (gps.time.minute() < 10) Serial.print("0");
        Serial.print(gps.time.minute());
        Serial.print(":");
        if (gps.time.second() < 10) Serial.print("0");
        Serial.println(gps.time.second());
      } else {
        Serial.println("Time: Not Available");
      }
      Serial.println();
    }
  }

  if (micros() - LoopTimer2 > 400000) {
    // Toggle LED state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // Reset LoopTimer for the next iteration
    LoopTimer2 = micros();
  }  
}
