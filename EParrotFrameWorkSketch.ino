/*-----( Feature Definitions )-----*/
//#define FEATURE_ENABLED_ADAFRUIT_BMP180
//#define FEATURE_ENABLED_SPARKFUN_BMP180

// Adafruit BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
#include <Wire.h>
#include <Adafruit_Sensor.h>            // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BMP085_U.h>          // https://github.com/adafruit/Adafruit-BMP085-Library
#endif

// Sparkfun BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_SPARKFUN_BMP180
#include <Wire.h>
#include <SFE_BMP180.h>			// https://github.com/sparkfun/BMP180_Breakout
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
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // BMP180 pressure sensor
#endif

// Sparkfun BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_SPARKFUN_BMP180
SFE_BMP180 bmp;  // BMP180 pressure sensor
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

  doFunctionAtInterval(readTemperatureSensors, &lastTemperatureRead, READ_TEMPERATURE_SENSORS_EVERY);  // read temperature sensors
  doFunctionAtInterval(readPressureSensors, &lastPressureRead, READ_PRESSURE_SENSORS_EVERY);  // read pressure sensors
  doFunctionAtInterval(readUserInput, &lastUserInput, READ_USER_INPUT_EVERY);  // read user input
  doFunctionAtInterval(writeDisplay, &lastDisplayWrite, WRITE_DISPLAY_EVERY);  // write values to display

}

void readTemperatureSensors() {

  // Read temperature sensor values

}

void initPressureSensors() {

  // Set pressure for each temperature sensor to default pressure
  for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
    temperatureSensors[i].tToABV.Pressure(defaultPressure);
  }

#if (defined(FEATURE_ENABLED_ADAFRUIT_BMP180) || defined(FEATURE_ENABLED_SPARKFUN_BMP180))
  if (!bmp.begin()) {
    // There was a problem detecting the BMP180 ... check your connections
    serialDivider();
    Serial.println("No BMP180 detected ... Check your wiring or I2C ADDR!");
    serialDivider();
  } else readPressureSensors();

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

#ifdef FEATURE_ENABLED_SPARKFUN_BMP180
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = bmp.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = bmp.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = bmp.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // Function returns 1 if successful, 0 if failure.

        status = bmp.getPressure(P, T);
        if (status != 0)
        {
          // Success - do loop here
          for (int i = 0; i < TOTAL_TEMPERATURE_SENSORS; i++) {
            temperatureSensors[i].tToABV.Pressure(P);
          }
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

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
