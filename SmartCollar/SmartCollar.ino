/* Adafruit library examples for BME280 sensor */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoBLE.h>
/* If you know the sea level pressure, the accuracy improves a lot
#define SEALEVELPRESSURE_HPA (1013.25) */
#define LSL8 256 

Adafruit_BME280 bme;
Adafruit_BME280 bme2;

BLEService tempService("98a06868-7df4-431d-b940-8d92656e7893"); // BLE LED Service

//BLEIntCharacteristic internalTemp("c860de29-0df7-4832-8c9f-cb8b6bdac985", BLEWrite | BLENotify);
BLEIntCharacteristic externalTemp("3285a9d7-857b-4233-809e-67e077a0366c", BLEWrite | BLENotify);

BLEIntCharacteristic combinedTemp("c860de29-0df7-4832-8c9f-cb8b6bdac985", BLEWrite | BLENotify);

long prevMillis = 0;
int dogSafeTemp = 0;

int data[] = {95,96,97,98,99,100,99,98,97,96};

void setup() {
  Serial.begin(9600);
  // while (!Serial);
  Serial.println("Starting");
  bme.begin(0x77); // address = 0x77 (default)
  bme2.begin(0x76); // address = 0x76 

/* assuming I2C address set is 0x76. If you are using 0x77, update 0x77 as the I2C address  */

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  Serial.println("Starting");
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  
  // if (!bme.begin(0x76)) {
  //   Serial.println("No BME280 device found!");
  //   while (1);
  // }else if (!bme.begin(0x77)) {
  //   Serial.println("No BME280 device found!");
  //   while (1);
  // }

  Serial.println("Starting BLE");

  BLE.setLocalName("SmartCollar");
  BLE.setAdvertisedService(tempService);
  // tempService.addCharacteristic(internalTemp);
  // tempService.addCharacteristic(externalTemp);
  tempService.addCharacteristic(combinedTemp);
  BLE.addService(tempService);
  //internalTemp.writeValue(dogSafeTemp); // set initial value for this characteristic

  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {

  BLEDevice central = BLE.central();
  int i = 0;
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
    long currMillis = millis();

      if(currMillis - prevMillis >= 1000){
        prevMillis = millis();
        //updateTemp();
        //test(sel);
        //updateTemps();

        test2(i);
        i += 1;
        if(i == 10){
          i = 0;
        }
      }
    }  
  
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateTemps() {
  Serial.print("Temperature in deg F = ");
  double cel = bme.readTemperature();
  int internalTempData = (int)((cel*9/5) + 32);
  Serial.println(internalTempData);
  //internalTemp.writeValue(internalTempData);

  Serial.print("Temperature in deg F2 = ");
  double cel2 = bme2.readTemperature();
  int externalTempData = (int)((cel2*9/5) + 32);
  Serial.println(externalTempData);
  //externalTemp.writeValue(externalTempData);

  int combinedTempData = externalTempData * LSL8 + internalTempData;
  combinedTemp.writeValue(combinedTempData);
}

void test(bool sel){
  int rand = random(0, 120);
  int rand2 = random(0, 120);
  
  int combine = rand + rand2 * 256;
  Serial.print("Internal: ");
  Serial.println(rand);
  Serial.print("External: ");
  Serial.println(rand2);
  Serial.print("Combine: ");
  Serial.println(combine);
  Serial.println();
  combinedTemp.writeValue(combine);  
  
  // Serial.print("External: ");
  // Serial.println(rand2);
  // externalTemp.writeValue(rand2);  

  
  //   Serial.println("Internal: 9652");

  //   internalTemp.writeValue(10112);  
}

void test2(int i){
  Serial.println(data[i]);

  int combine = data[i] + (data[i] + 1) * LSL8;
  combinedTemp.writeValue(combine);
}
