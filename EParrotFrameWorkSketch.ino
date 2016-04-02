/*-----( Feature Definitions )-----*/
//#define FEATURE_ENABLED_ADAFRUIT_BMP180

// Adafruit BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
#include <Adafruit_Sensor.h>            // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BMP085_U.h>          // https://github.com/adafruit/Adafruit-BMP085-Library
#endif

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
const float defaultPressure = 1013.25;

/*-----( Declare objects )-----*/
struct temperatureSensor {
  //temperatureSensorClassObject sensor;	// Declare sensor object here e.g. DS18B20 or SMT172 or PT-100
  String sensorName;				// Sensor Name
  TToABV tToABV; 				// Instance or ToToAVB.h
  int state;        				// Input for calculating LIQUID or VAPOR ABV
};

#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP180_ID); // BMP180 pressure sensor
#endif

/*-----( Declare variables )-----*/
// Declare array of sensors
temperatureSensor temperatureSensors[TOTAL_TEMPERATURE_SENSORS];

//Declare asynchronous function last event times
unsigned long lastTemperatureRead;
unsigned long lastPressureRead;
unsigned long lastUserInput;
unsigned long lastDisplayWrite;

void setup() {

  Serial.begin(115200);

  // Initialise non-looping temperature sensor values
  int sensorID; // Manually change for each sensor added
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

  // Initialise pressure sensor
  initPressureSensors();

  // Initialise asynchronous function variables
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

void initPressureSensors() {

  // Set pressure for each temperature sensor to default pressure
  for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
    temperatureSensors[i].tToABV.Pressure(defaultPressure);
  }

#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
  if (!bmp.begin()) {
    // There was a problem detecting the BMP085 ... check your connections
    serialDivider();
    Serial.println("No BMP085 detected ... Check your wiring or I2C ADDR!");
    serialDivider();
  }
#endif

}

void readPressureSensors() {

#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
  sensors_event_t event;

  bmp.getEvent(&event);
  if (event.pressure) {
    for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
      temperatureSensors[i].tToABV.Pressure(event.pressure);
    }
  }
#endif

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

  Serial.println("---------------------------------------");

}

