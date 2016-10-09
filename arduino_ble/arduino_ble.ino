


#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_UART.h"

#define RTS  10
#define RXI  11
#define TXO  12
#define CTS  13
#define MODE -1

#define BUFSIZE 128
#define VERBOSE_MODE false
Adafruit_BNO055 bno = Adafruit_BNO055(55);
SoftwareSerial bluefruitSS = SoftwareSerial(TXO, RXI);
Adafruit_BluefruitLE_UART ble(bluefruitSS, MODE, CTS, RTS);

int battery = 0;
int count = 0;

void setup() {

  Serial.begin(115200);
  Serial.println(F("BLE Starting"));

  delay(5000);

  if (! ble.begin(VERBOSE_MODE)) {
    Serial.println( F("FAILED!") );
    while (1);
  }

  Serial.println( F("OK!") );

  ble.echo(false);

  Serial.print(F("Set device name: "));
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=BNOIOS"))) {
    Serial.println(F("FAILED."));
    while (1);
  }

  Serial.println(F("OK!"));

  ble.reset();

  Serial.print(F("BNO055 init: "));
  if (! bno.begin()) {
    Serial.println(F("FAILED."));
    while (1);
  }
  
  delay(1000);
  
  Serial.println(F("OK 2!"));

  bno.setExtCrystalUse(true);

  while (! ble.isConnected())
    delay(500);

  Serial.println(F("Connected"));

  //batteryLevel();

}

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);

  uint8_t system, gyro, accel, mag = 0;
  
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(gyro, DEC);
 
  ble.print("AT+BLEUARTTX=");
  //char buf[40];
  //char xaxis[6];
  //char z[6];
  //dtostrf(event.orientation.x, sizeof xaxis, 1, xaxis);
  //dtostrf(event.orientation.z, sizeof z, 1, z);
  //int nchars = snprintf(buf, sizeof buf, "x %d | z %s | n ", event.orientation.x, z);
  //ble.print(buf);
  //ble.println(nchars);
  
  //ble.flush();
  
  ble.print("[");
  // !seems system output here breaks everyting. Or else I got unlucky w multiple bootloads
  //ble.print(system, DEC);
  //ble.print(",");
  ble.print(gyro, DEC);
  ble.print(accel, DEC);
  ble.print(mag, DEC);
  ble.print(",");
  ble.print(event.orientation.x, 1);
  ble.print(",");
  ble.print(event.orientation.y, 1);
  ble.print(",");
  ble.print(event.orientation.z, 1);
  //ble.print(",");
  //ble.print(battery, DEC);
  ble.println("]");

  
  ble.readline();

  if (count == 5000) {
    batteryLevel();
    count = 0;
  } else {
    delay(200);
    count++;
  }

}

void batteryLevel() {

  ble.println("AT+HWVBAT");
  ble.readline(1000);

  if (strcmp(ble.buffer, "OK") == 0) {
    battery = 0;
  } else {
    battery = atoi(ble.buffer);
    ble.waitForOK();
  }

}

