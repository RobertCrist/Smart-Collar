/* Adafruit library examples for BME280 sensor */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/* If you know the sea level pressure, the accuracy improves a lot
#define SEALEVELPRESSURE_HPA (1013.25) */

Adafruit_BME280 bme;
Adafruit_BME280 bme2;


void setup() {
  Serial.begin(9600);
/* assuming I2C address set is 0x76. If you are using 0x77, update 0x77 as the I2C address  */
  while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));
  if (!bme.begin(0x76)) {
    Serial.println("No BME280 device found!");
    while (1);
  }else if (!bme.begin(0x77)) {
    Serial.println("No BME280 device found!");
    while (1);
  }
  bme.begin(0x77); // address = 0x77 (default)
  bme2.begin(0x76); // address = 0x76 
}

void loop() {

/* read all the parameters from the BME280 sensors via bme object created*/

  Serial.print("Temperature in deg F = ");
  double cel = bme.readTemperature();
  double temp = (cel*9/5) + 32;
  Serial.println(temp);

  Serial.print("Temperature in deg F2 = ");
  double cel2 = bme2.readTemperature();
  double temp2 = (cel2*9/5) + 32;
  Serial.println(temp2);
/*
  Serial.print("Pressure in hPa = ");
  Serial.println(bme.readPressure() / 100.0F);
*/

  Serial.println();
  delay(5000);
}