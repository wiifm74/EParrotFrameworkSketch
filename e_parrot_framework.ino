#include "myBoard.h"

/*-----( Definitions)-----*/
#define READ_TEMPERATURE_SENSORS_EVERY 400
#define READ_PRESSURE_SENSORS_EVERY 300000
#define READ_USER_INPUT_EVERY 20
#define WRITE_DISPLAY_EVERY 250

/*-----( Declare objects )-----*/

/*-----( Declare constants )-----*/

/*-----( Declare variables )-----*/

//Declare asynchronous function last event times
unsigned long lastTemperatureRead;
unsigned long lastPressureRead;
unsigned long lastUserInput;
unsigned long lastDisplayWrite;	

void setup() {

  Serial.begin(115200);

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

	// Read pressure sensor values
		
}

void readUserInput() {

	// Read user input
		
}

void writeDisplay() {

  // output to display

}

