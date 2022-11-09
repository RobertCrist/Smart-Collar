/* Adafruit library examples for BME280 sensor */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoBLE.h>
/* If you know the sea level pressure, the accuracy improves a lot
#define SEALEVELPRESSURE_HPA (1013.25) */

Adafruit_BME280 bme;
Adafruit_BME280 bme2;

BLEService tempService("180A"); // BLE LED Service
BLEDoubleCharacteristic internalTemp("2A19", BLERead | BLENotify);
BLEDoubleCharacteristic externalTemp("2A20", BLERead | BLENotify);

long prevMillis = 0;
int dogSafeTemp = 0;
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting");
  bme.begin(0x77); // address = 0x77 (default)
  bme2.begin(0x76); // address = 0x76 

/* assuming I2C address set is 0x76. If you are using 0x77, update 0x77 as the I2C address  */

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  pinMode(6, OUTPUT);
  Serial.println("Starting");
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  
  if (!bme.begin(0x76)) {
    Serial.println("No BME280 device found!");
    while (1);
  }else if (!bme.begin(0x77)) {
    Serial.println("No BME280 device found!");
    while (1);
  }
  Serial.println("Starting BLE");

  BLE.setLocalName("SmartCollar");
  BLE.setAdvertisedService(tempService);
  tempService.addCharacteristic(internalTemp);
  BLE.addService(tempService);
  internalTemp.writeValue(dogSafeTemp); // set initial value for this characteristic

  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {

/* read all the parameters from the BME280 sensors via bme object created*/

  BLEDevice central = BLE.central();

  //while (central.connected()) {
    long currMillis = millis();

    if(currMillis - prevMillis >= 1000){
      prevMillis = millis();
      updateBatteryLevel();
      
    }

    if(currMillis % 500 == 0){
      digitalWrite(6, HIGH);
    }

    if(currMillis % 500 == 250){
      digitalWrite(6, LOW);
    }
  //}
  if (central) {
      Serial.print("Connected to central: ");
      // print the central's BT address:
      Serial.println(central.address());
      // turn on the LED to indicate the connection:
      digitalWrite(LED_BUILTIN, HIGH);

      // check the battery level every 200ms
      // while the central is connected:
      
        
    
      // when the central disconnects, turn off the LED:
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
  }
}

void updateBatteryLevel() {
  Serial.print("Temperature in deg F = ");
  double cel = bme.readTemperature();
  double temp = (cel*9/5) + 32;
  Serial.println(temp);

  Serial.print("Temperature in deg F2 = ");
  double cel2 = bme2.readTemperature();
  double temp2 = (cel2*9/5) + 32;
  Serial.println(temp2);
}


