#include <ArduinoBLE.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "GasBreakout.h"

#define DEBUG

DHT_Unified dht(9, DHT22);

GasBreakout gas(Wire, 0x19);

BLEService sensorService("180F");

// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",BLERead | BLENotify);
BLEFloatCharacteristic temperatureChar("FFF1",BLERead | BLENotify);
BLEFloatCharacteristic humidityChar("FFF2",BLERead | BLENotify);
BLEFloatCharacteristic redChar("FFF3",BLERead | BLENotify);
BLEFloatCharacteristic nh3Char("FFF4",BLERead | BLENotify);
BLEFloatCharacteristic oxChar("FFF5",BLERead | BLENotify);

int oldBatteryLevel = 0;  // last battery level reading from analog input
float oldTemperature = 0;
float oldHumidity = 0;
float oldReduction = 0;
float oldNh3 = 0;
float oldOxidation = 0;
long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  Serial.begin(115200);    // initialize serial communication
  Wire.begin();
  
  #ifdef DEBUG
  while (!Serial);
  #endif
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  //DHT22 
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  #ifdef DEBUG
    Serial.println(F("DHT22 Sensor"));
    Serial.println(F("------------------------------------"));
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(F("Error reading temperature!"));
    }
    else {
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println(F("Error reading humidity!"));
    }
    else {
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
    }
    Serial.println(F("------------------------------------"));
  #endif

  //MiCS6814
  if (!gas.initialise()) {
    Serial.println("MICS6814 - Not Initialized");
  }
  Serial.println("MICS6814");
  Serial.println(F("------------------------------------"));
  //wait 1 second
  delay(1000);
  GasBreakout::Reading reading;
  reading = gas.readAll();
  #ifdef DEBUG
    Serial.println("Reading Gas");
    Serial.print("Red: ");
    Serial.println(reading.reducing);
    Serial.print("NH3: ");
    Serial.println(reading.nh3);
    Serial.print("Ox: ");
    Serial.println(reading.oxidising);
    Serial.println("");
    Serial.println(F("------------------------------------"));
   #endif
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Winsource-sense");
  BLE.setAdvertisedService(sensorService); // add the service UUID
  sensorService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  sensorService.addCharacteristic(temperatureChar); // add the battery level characteristic
  sensorService.addCharacteristic(humidityChar); // add the battery level characteristic
  sensorService.addCharacteristic(redChar); // add the battery level characteristic
  sensorService.addCharacteristic(nh3Char); // add the battery level characteristic
  sensorService.addCharacteristic(oxChar); // add the battery level characteristic

  BLE.addService(sensorService); // Add the sensor service
  batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic
  temperatureChar.writeValue(0); // set initial value for this characteristic
  humidityChar.writeValue(0); // set initial value for this characteristic
  redChar.writeValue(0); // set initial value for this characteristic
  nh3Char.writeValue(0); // set initial value for this characteristic
  oxChar.writeValue(0); // set initial value for this characteristic
  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 2000ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 2000ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 2000) {
        previousMillis = currentMillis;
        updateCharacteristics();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateCharacteristics() {
  int battery = analogRead(A3);
  int batteryLevel = map(battery, 0, 1023, 0, 100); //calculate voltage div
  
  sensors_event_t event_temp;
  sensors_event_t event_hum;
  GasBreakout::Reading reading;
  reading = gas.readAll();
  
  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
  
  dht.temperature().getEvent(&event_temp);
  if (event_temp.temperature != oldTemperature) {      // if the temperature level has changed
    Serial.print("temperature is now: "); // print it
    Serial.println(event_temp.temperature);
    temperatureChar.writeValue(event_temp.temperature); 
    oldTemperature = event_temp.temperature;           // save the level for next comparison
  }
  
  dht.humidity().getEvent(&event_hum);
  if (event_hum.relative_humidity != oldHumidity) {      // if the humidity level has changed
    Serial.print("Humidity is now: "); // print it
    Serial.println(event_hum.relative_humidity);
    humidityChar.writeValue(event_hum.relative_humidity); 
    oldHumidity = event_hum.relative_humidity;           // save the level for next comparison
  }

  if (reading.reducing != oldReduction) {      // if the reducing sensor level has changed
    Serial.print("Red sensor is now: "); // print it
    Serial.println(reading.reducing);
    redChar.writeValue(reading.reducing); // set initial value for this characteristic
    oldReduction = reading.reducing;           // save the level for next comparison
  }

   if (reading.nh3 != oldNh3) {      // if the reducing sensor level has changed
    Serial.print("Red sensor is now: "); // print it
    Serial.println(reading.nh3);
    nh3Char.writeValue(reading.nh3);
    oldNh3 = reading.nh3;           // save the level for next comparison
  }

  if (reading.oxidising != oldOxidation) {      // if the reducing sensor level has changed
    Serial.print("Red sensor is now: "); // print it
    Serial.println(reading.oxidising);
    oxChar.writeValue(reading.oxidising);
    oldOxidation = reading.oxidising;           // save the level for next comparison
  }


  
}
