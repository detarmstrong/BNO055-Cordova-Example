


#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

#define RTS  10
#define RXI  11
#define TXO  12
#define CTS  13
#define MODE -1

#define BUFSIZE 128
#define VERBOSE_MODE false
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int battery = 0;
int count = 0;

void setup() {

  Serial.begin(115200);
  
  Serial.println(F("Waiting 5 seconds!"));

  delay(1000);

  Serial.print(F("BNO055 init: "));
  if (! bno.begin()) {
    Serial.println(F("FAILED."));
    while (1);
  }
  
  delay(1000);

  bno.setExtCrystalUse(true);
  Serial.println(F("Finished setup"));
}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);

  uint8_t system, gyro, accel, mag = 0;
  
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(gyro, DEC);
  Serial.print(accel, DEC);
  Serial.print(mag, DEC);
  Serial.print(",");
  Serial.print(event.orientation.x, 1);
  Serial.print(",");
  Serial.print(event.orientation.y, 1);
  Serial.print(",");
  Serial.print(event.orientation.z, 1);
  Serial.println("");

}

