#include "TracerMPPT.h"
#include <SoftwareSerial.h>
#include <stdint.h>

TracerMPPT tracer(2,3); //parameters are rx, tx pins, in that order.
  
void setup() {
  Serial.begin(57600);
  Serial.println("Welcome.");
  tracer.begin();
  Serial.println("Initialized.");
}

void printResult(ManualControlResult* result) {
  if (result->isDataValid) {
    Serial.println("V=========== VALID =========V");
  } else {
    Serial.println("V*** BAD **** BAD **** BAD ***V  <<<<<<<<<");
  }

    
  Serial.print("Is Valid? "); 
  Serial.print(result->isDataValid);
  Serial.print(" : ");
  Serial.println(result->errorCode); //TODO: add error message from errorcode
  
  
  Serial.print("Load On? "); 
  Serial.println(result->isLoadOn);

  if (result->isDataValid) {
    Serial.println("^=========== VALID =========^");
  } else {
    Serial.println("^*** BAD **** BAD **** BAD ***^");
  }
}

void printData(TracerData* data) {
  if (data->isDataValid) {
    Serial.println("V=========== VALID =========V");
  } else {
    Serial.println("V*** BAD **** BAD **** BAD ***V  <<<<<<<<<");
  }

  
  Serial.print("Is Valid? "); 
  Serial.print(data->isDataValid);
  Serial.print(" : ");
  Serial.println(data->errorCode); //TODO: add error message from errorcode
  
  
  Serial.print("Battery Voltage: "); 
  Serial.println(data->batteryVoltage);

  Serial.print("PV Voltage: "); 
  Serial.println(data->pvVoltage);

  Serial.print("Load Current: "); 
  Serial.println(data->loadCurrent);

  Serial.print("Over Dischage: "); 
  Serial.println(data->overDischarge);

  Serial.print("Battery Max: "); 
  Serial.println(data->batteryMax);

  Serial.print("Load On? "); 
  Serial.println(data->isLoadOn);

  Serial.print("Overload? "); 
  Serial.println(data->isOverload);

  Serial.print("Load Short? "); 
  Serial.println(data->isLoadShort);

  Serial.print("Battery Overload? "); 
  Serial.println(data->isBatteryOverload);

  Serial.print("Battery Overdischage? "); 
  Serial.println(data->isBatteryOverDischarge);

  Serial.print("Battery Full? "); 
  Serial.println(data->isBatteryFull);

  Serial.print("Battery Charging? "); 
  Serial.println(data->isBatteryCharging);

  Serial.print("Temperature: "); 
  Serial.println(data->batteryTemperature);

  Serial.print("Charge Current: "); 
  Serial.println(data->chargeCurrent);
  
  if (data->isDataValid) {
    Serial.println("^=========== VALID =========^");
  } else {
    Serial.println("^*** BAD **** BAD **** BAD ***^");
  }
}

void loop() {
  
  TracerData data;
  ManualControlResult ctrlResult;
  
  tracer.getRealtimeData(&data);
  printData(&data);  

  delay(5000);
  
  Serial.println("SETTING LOAD ---- OFF ----");
  tracer.setLoad(false, &ctrlResult);
  printResult(&ctrlResult);

  tracer.getRealtimeData(&data);
  printData(&data);
  
  delay(2000);
  
  Serial.println("SETTING LOAD **** ON ****");
  tracer.setLoad(true, &ctrlResult);
  printResult(&ctrlResult);
}

