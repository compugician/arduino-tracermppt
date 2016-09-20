#include "TracerMPPT.h"
#include <SoftwareSerial.h>
#include <stdint.h>

TracerMPPT tracer(2,3); //parameters are rx, tx pins, in that order.
  
void setup() {
  Serial.begin(115200);
  Serial.println("Welcome.");
  tracer.begin();
  Serial.println("Initialized.");
}

void printResult(ManualControlResult* result) {
  if (result->validity.isDataValid) {
    Serial.println("V=========== VALID =========V");
  } else {
    Serial.println("V*** BAD **** BAD **** BAD ***V  <<<<<<<<<");
  }

  Serial.print("Is Valid? "); 
  Serial.print(result->validity.isDataValid);
  Serial.print(" [");
  Serial.print(result->validity.errorString);
  Serial.println("]");
  
  Serial.print("Load On? "); 
  Serial.println(result->isLoadOn);

  if (result->validity.isDataValid) {
    Serial.println("^=========== VALID =========^");
  } else {
    Serial.println("^*** BAD **** BAD **** BAD ***^");
  }
}

void printData(TracerData* data) {
  if (data->validity.isDataValid) {
    Serial.println("V=========== VALID =========V");
  } else {
    Serial.println("V*** BAD **** BAD **** BAD ***V  <<<<<<<<<");
  }

  
  Serial.print("Is Valid? "); 
  Serial.print(data->validity.isDataValid);
  Serial.print(" [");
  Serial.print(data->validity.errorString); //TODO: add error message from errorcode
  Serial.println("]");
  
  
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
  
  if (data->validity.isDataValid) {
    Serial.println("^=========== VALID =========^");
  } else {
    Serial.println("^*** BAD **** BAD **** BAD ***^");
  }
}

void printErrorCounts() {
  uint16_t* errorCounts = tracer.getErrorCounts();
  uint8_t minErrorValue = tracer.getMinErrorValue();
  Serial.println("Error counts:");
  for (int i = 0 ; i<tracer.getTotalNumberOfErrorTypes(); i++) {
    Serial.print(minErrorValue+i,DEC);
    Serial.print(" ");
    Serial.println(errorCounts[i],DEC);
  }
  Serial.print("# of Reads: ");
  Serial.println(tracer.getTotalNumberOfReads());
}

void loop() {
  
  TracerData data;
  ManualControlResult ctrlResult;
  
  tracer.getRealtimeData(&data);
  printData(&data);  
  
  delay(5);
  
  Serial.println("SETTING LOAD ---- OFF ----");
  tracer.setLoad(false, &ctrlResult);
  printResult(&ctrlResult);
  
  tracer.getRealtimeData(&data);
  printData(&data);
   
  delay(200);
  delay(random(50,100));
  
  Serial.println("SETTING LOAD **** ON ****");
  tracer.setLoad(true, &ctrlResult);
  printResult(&ctrlResult);

  printErrorCounts();
  
}

