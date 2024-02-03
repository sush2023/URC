// GPS Testing - Kaegan Phillips (1/24/24) 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define RX_PIN 0
#define TX_PIN 1

SoftwareSerial serial_connection(RX_PIN, TX_PIN);
TinyGPSPlus gps;  // instantiate GPS object

void setup()
{
  Serial.begin(9600); // opens communications to Serial Monitor in Arduino IDE
  serial_connection.begin(4800); // opens communication to GPS
  Serial.println("GPS Start"); // show in Serial Monitor that the program has started
}

void loop()
{
  // while there are characters coming from the GPS
  while(serial_connection.available())
  {
    gps.encode(serial_connection.read()); // feeds serial NMEA data into the library one char at a time
  }
  // if NMEA package has arrived 
  if(gps.location.isUpdated())
  {
    // get satellite count
    Serial.println("Satellite Count:");
    Serial.println(gps.satellites.value());
    // get latitude value
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    // get longitude value
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 6);
    // get speed (mph or mps)
    Serial.println("Speed (mph):");
    Serial.println(gps.speed.mph());
    // get altitude (feet or meter)
    Serial.println("Altitude (feet):");
    Serial.println(gps.altitude.feet());
    Serial.println("");
  }
}