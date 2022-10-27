/* Adafruit library examples for BME280 sensor */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoBLE.h>
#include <ArduinoBLE.h>
/* If you know the sea level pressure, the accuracy improves a lot
#define SEALEVELPRESSURE_HPA (1013.25) */

Adafruit_BME280 bme;
Adafruit_BME280 bme2;

BLEService batteryService("180F");

void setup() {
  Serial.begin(9600);
  if (!bme.begin(0x76)) {
    Serial.println("No BME280 device found!");
    while (1);
  }
}

void loop() {

  Serial.print("Temperature in deg F = ");
  double cel = bme.readTemperature();
  double outerTemp = (cel*9/5) + 32;
  Serial.println(outerTemp);

  Serial.print("Temperature in deg F2 = ");
  double cel2 = bme2.readTemperature();
  double innterTemp = (cel2*9/5) + 32;
  Serial.println(innterTemp);

  Serial.println();
  delay(5000);
}
