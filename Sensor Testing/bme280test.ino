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
  if (!bme.begin(0x76)) {
    Serial.println("No BME280 device found!");
    while (1);
  }
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

/*
  Serial.println("Altitude in m = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));

  Serial.print("Humidity in %RH = ");
  Serial.println(bme.readHumidity());
*/
  Serial.println();
  delay(5000);
}
