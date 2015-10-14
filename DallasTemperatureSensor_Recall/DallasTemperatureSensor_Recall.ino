/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Enhanced by LeoDesigner https://github.com/leodesigner
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 * 
 * 
 */

#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SimpleTimer.h>

#define ONE_WIRE_BUS              3      // Pin where dallase sensor is connected 
#define COMPARE_TEMP              1      // Send temperature only if changed? 1 = Yes 0 = No
#define MAX_ATTACHED_DS18B20      16
#define EEPROM_DEVICE_ADDR_START  64     // start byte in eeprom for remembering our sensors
#define EEPROM_DEVICE_ADDR_END    EEPROM_DEVICE_ADDR_START+MAX_ATTACHED_DS18B20*2

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 

float lastTemperature[MAX_ATTACHED_DS18B20];
uint8_t numSensors = 0;
uint8_t currentTsensor = 0;
DeviceAddress dsaddr[MAX_ATTACHED_DS18B20];
bool ts_spot[MAX_ATTACHED_DS18B20];
// Initialize temperature message
MyMessage msg(0,V_TEMP);

MySensor gw;
SimpleTimer timer;

boolean receivedConfig = false;
boolean metric = true; 

void setup()  
{ 
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin();

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Temperature Sensor", "1.2");

  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     // reset used spot array
     ts_spot[i] = false;
  }
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     sensors.getAddress(dsaddr[i],i);
     // check if we know this sensor
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx >= 0) {
        // we know this sensor
        ts_spot[sidx] = true;
     }
  }
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx < 0) {
        // we have unknown sensor (not present in EEPROM)
        // let's find a first free spot and put the addr hash there
        uint8_t spot = 0;
        while (ts_spot[spot] && spot < MAX_ATTACHED_DS18B20) {
          spot++;
        }
        ts_spot[spot] = true;
        storeSensorAddr(dsaddr[i],spot);
        sidx = spot;
        #ifdef NDEBUG
          Serial.print(F("Added new sensor to spot # "));
          Serial.println(spot);
        #endif
     }
     
     gw.present(sidx, S_TEMP);
     #ifdef NDEBUG
       Serial.println();
       Serial.print(i);
       Serial.print(F(" index: "));
       Serial.print(sidx);
       Serial.print(F(" address: "));
       printAddress(dsaddr[i]);
       Serial.println();
     #endif
  }

  // start our periodic jobs
  // many other periodic jobs can be added here
  timer.setInterval(60000, checkTemperature);

}

void loop() {
  timer.run();
  gw.process();
}


// a simple hash function for 1wire address to reduce id to two bytes
uint16_t simpleAddrHash(DeviceAddress a){
  return ((a[1] ^ a[2] ^ a[3] ^ a[4] ^ a[5] ^ a[6]) << 8) + a[7];
}

// search for device address hash in eeprom
// return -1 if not found
int8_t getSensorIndex(DeviceAddress a) {
  uint16_t hash = simpleAddrHash(a);
  int8_t idx = -1;
  uint8_t aidx = 0;
  uint8_t ptr = EEPROM_DEVICE_ADDR_START;
  while (ptr < EEPROM_DEVICE_ADDR_END && idx == -1) {
    uint8_t hash1 = gw.loadState(ptr);
    uint8_t hash2 = gw.loadState(ptr+1);
    if ( hash1 == (uint8_t)(hash >> 8) && hash2 == (uint8_t)(hash & 0x00FF)) {
      // found device index
      idx = aidx;
    }
    aidx++;
    ptr+=2;
  }
  return idx;
}

// save address hash in EEPROM under index
void storeSensorAddr(DeviceAddress a, uint8_t index) {
    uint16_t hash = simpleAddrHash(a);
    uint8_t ptr = EEPROM_DEVICE_ADDR_START + index * 2;
    if (ptr < EEPROM_DEVICE_ADDR_END) {
      gw.saveState(ptr,   hash >> 8);
      gw.saveState(ptr+1, hash & 0x00FF);
    }
    #ifdef NDEBUG
      Serial.print(F("storeSensorAddr under index: "));
      Serial.println(index);
    #endif
}



void checkTemperature(){
  // Check temperature
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  
  if (numSensors > 0) {
    currentTsensor = 0;
    timer.setTimeout(conversionTime, readTemperature);
  }
}

void readTemperature(){

  #ifdef NDEBUG
  unsigned long start = millis();
  #endif

  // Fetch and round temperature to one decimal
  //float temperature = static_cast<float>(static_cast<int>(sensors.getTempC(dsaddr[currentTsensor]) * 10.)) / 10.;
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempC(dsaddr[currentTsensor]):sensors.getTempF(dsaddr[currentTsensor])) * 10.)) / 10.;

  // Only send data if temperature has changed and no error
  #if COMPARE_TEMP == 1
  if (lastTemperature[currentTsensor] != temperature && temperature != -127.00 && temperature != 85.00) {
  #else
  if (temperature != -127.00 && temperature != 85.00) {
  #endif
    // Send in the new temperature
    gw.send(msg.setSensor(getSensorIndex(dsaddr[currentTsensor])).set(temperature,1));
    // Save new temperatures for next compare
    lastTemperature[currentTsensor] = temperature;
  }
  #ifdef NDEBUG
  Serial.print(F("Temperature "));
  Serial.print(currentTsensor,DEC);
  Serial.print(F(" index: "));
  Serial.print(getSensorIndex(dsaddr[currentTsensor]));
  Serial.print(F(" -> "));
  Serial.print(temperature);
  unsigned long etime = millis() - start;
  Serial.print(F(" time elapsed: "));
  Serial.println(etime);
  #endif
  
  currentTsensor++;
  if (currentTsensor < numSensors && currentTsensor < MAX_ATTACHED_DS18B20) {
    // postpone next sensor reading
    timer.setTimeout(25, readTemperature);    
  }
}


#ifdef NDEBUG
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
#endif


