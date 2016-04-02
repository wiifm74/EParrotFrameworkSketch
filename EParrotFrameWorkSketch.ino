#include <TToABV.h>
#include "myBoard.h"

/*-----( Definitions )-----*/
#define LIQUID 1
#define VAPOR 2
#define TOTAL_TEMPERATURE_SENSORS 2

#define READ_TEMPERATURE_SENSORS_EVERY 400
#define READ_PRESSURE_SENSORS_EVERY 300000
#define READ_USER_INPUT_EVERY 20
#define WRITE_DISPLAY_EVERY 250

/*-----( Declare constants )-----*/


/*-----( Declare objects )-----*/
struct temperatureSensor {
  //temperatureSensorClassObject sensor;	// Declare sensor object here e.g. DS18B20 or SMT172 or PT-100
  String sensorName;				// Sensor Name
  TToABV tToABV; 				// Instance or ToToAVB.h
  int state;        				// Input for calculating LIQUID or VAPOR ABV
};

/*-----( Declare variables )-----*/
// Declare array of sensors
temperatureSensor temperatureSensors[TOTAL_TEMPERATURE_SENSORS];

//Declare asynchronous function last event times
unsigned long lastTemperatureRead;
unsigned long lastPressureRead;
unsigned long lastUserInput;
unsigned long lastDisplayWrite;

void setup() {

  int sensorID;

  Serial.begin(115200);

  // Initialise non-looping temperature sensor values
  // Column sensor
  sensorID = 0;
  temperatureSensors[sensorID].sensorName = "Column";
  temperatureSensors[sensorID].state = VAPOR;
  // Boiler sensor
  sensorID = 1;
  temperatureSensors[sensorID].sensorName = "Boiler";
  temperatureSensors[sensorID].state = LIQUID;

  for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
    // Initialise looping/repeated temperature sensor values

    // set tToABV object to calculate correct ABV values
    switch (temperatureSensors[i].state) {
      case VAPOR:
        temperatureSensors[1].tToABV.Vapor();
        break;
      case LIQUID:
        temperatureSensors[1].tToABV.Liquid();
        break;
      default:
        // something is wrong, set to vapor
        temperatureSensors[1].tToABV.Vapor();
        break;
    }
  }

  //Initialise asynchronous function variables
  unsigned long now = millis();
  lastTemperatureRead = now;
  lastPressureRead = now;
  lastUserInput = now;
  lastDisplayWrite = now;

}

void doFunctionAtInterval(void (*callBackFunction)(), unsigned long *lastEvent, unsigned long Interval) {

  unsigned long now = millis();

  if ((now - *lastEvent) >= Interval) {
    callBackFunction();
    *lastEvent = now;
  }

}

void loop() {

  doFunctionAtInterval(readTemperatatureSensors, &lastTemperatureRead, READ_TEMPERATURE_SENSORS_EVERY);  // read temperature sensors
  doFunctionAtInterval(readPressureSensors, &lastTemperatureRead, READ_PRESSURE_SENSORS_EVERY);  // read pressure sensors
  doFunctionAtInterval(readUserInput, &lastTemperatureRead, READ_USER_INPUT_EVERY);  // read user input
  doFunctionAtInterval(writeDisplay, &lastDisplayWrite, WRITE_DISPLAY_EVERY);  // write values to display

}

void readTemperatatureSensors() {

  // Read temperature sensor values

}

void readPressureSensors() {

  // Set pressure for each temperature sensor to default 1013.25mbars
  for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
    temperatureSensors[i].tToABV.Pressure(1013.25);
  }

}

void readUserInput() {

  // Read user input

}

void introDisplay() {

  // write introduction to serial
  introSerial();

  //write introduction to display

}

void writeDisplay() {

  // write values to Serial
  writeSerial();

  // write values to display

}

void introSerial() {

  Serial.println("Vision Stills e-Parrot Framework Sketch");
  serialDivider();

}

void writeSerial() {

  for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
    Serial.println(temperatureSensors[i].sensorName);
    Serial.print("Temperature: "); Serial.print(temperatureSensors[i].tToABV.Temperature(), 2); Serial.println("c");
    Serial.print("Pressure: "); Serial.print(temperatureSensors[i].tToABV.Pressure(), 2);  Serial.println("mbar");
    Serial.print("ABV: "); Serial.print(temperatureSensors[i].tToABV.ABV(), 2); Serial.println("%");
    serialDivider();
  }

}

void serialDivider() {
	
  Serial.println("/-------------------------------------/");
  
}

