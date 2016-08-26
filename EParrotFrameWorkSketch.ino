/*-----( Feature Definitions )-----*/
//#define FEATURE_ENABLED_ADAFRUIT_BMP180
//#define FEATURE_ENABLED_SPARKFUN_BMP180
//#define FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
//#define FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
#define FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR

#include <EEPROMex.h>			// https://github.com/thijse/Arduino-EEPROMEx
#include "TToABV.h"			// https://github.com/VisionStills/TemperatureToABV
#include "myBoard.h"

// Adafruit BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
#include <Wire.h>			// Included in Arduino
#include <Adafruit_Sensor.h>            // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BMP085_U.h>          // https://github.com/adafruit/Adafruit-BMP085-Library
#endif

// Sparkfun BMP180 pressure sensor library
#ifdef FEATURE_ENABLED_SPARKFUN_BMP180
#include <Wire.h>			// Included in Arduino
#include <SFE_BMP180.h>			// https://github.com/sparkfun/BMP180_Breakout
#endif

// DS18B20 temperature sensor library
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
#include <OneWire.h>			// https://github.com/bigjosh/OneWireNoResistor
#include <DallasTemperature.h>		// https://github.com/milesburton/Arduino-Temperature-Control-Library
#endif

// SMT172 temperature sensor library
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
#include <SMT172.h>
//using SMT172;
#endif

// Protovoltaics shield with PT-100 sensor library
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
#include <Wire.h>                       // Included in Arduino
#include <PV_RTD_RS232_RS485_Shield.h>  // http://prods.protovoltaics.com/rtd-rs232-rs485/lib/PV_RTD_RS232_RS485_Shield.zip
#endif

/*-----( Definitions )-----*/
#define LIQUID 1
#define VAPOR 2

#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
#define MAXIMUM_TEMPERATURE_SENSORS 10
#define TEMPERATURE_PRECISION 12
#endif

#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
#define MAXIMUM_TEMPERATURE_SENSORS 2
#define SMT172_SELECT 15		// Putting this here for now - it most likely belongs in myBoard.h
#define BUSY 0
#define SUCCESS 1
#define NOT_CONNECTED 2
#endif

#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
#define MAXIMUM_TEMPERATURE_SENSORS 4
#define RTD_SENSOR_WIRES 3              // Number of wires on RTD sensors - 2, 3 or 4
#define RTD_DRIVE_CURRENT 0.000250      // Set the RTD drive current to 250uA
#define RTD_PGA 32                      // A PGA value of 32 will allow measurements up to 463.5 deg C
#define RTD_SAMPLE_FREQUENCY 20
#endif

#define READ_TEMPERATURE_SENSORS_EVERY 400
#define READ_PRESSURE_SENSORS_EVERY 300000
#define READ_USER_INPUT_EVERY 20
#define WRITE_DISPLAY_EVERY 250

/*-----( Declare constants )-----*/
const float defaultPressure = 1013.25;

const int CONFIG_VERSION = 1.0;
const int memoryBase = 32;

/*-----( Declare objects )-----*/
struct TemperatureSensor {
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
  DeviceAddress address;		// DS18B20 sensor address
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
  int select172;
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
  int rtdChannel;       		// Channel value for RTD shield
#endif
  String sensorName;			// Sensor Name
  TToABV tToABV; 			// Instance or ToToAVB.h
  int state;        			// Input for calculating LIQUID or VAPOR ABV
};

#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // BMP180 pressure sensor
#endif
#ifdef FEATURE_ENABLED_SPARKFUN_BMP180
SFE_BMP180 bmp;  // BMP180 pressure sensor
#endif

#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR

#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
PV_RTD_RS232_RS485 rtds(0x52, 100.0);	// RTD shield with PT-100 sensors
#endif

struct Settings {
  int version;
  int sensorsUsed;
  TemperatureSensor sensors[MAXIMUM_TEMPERATURE_SENSORS];
} settings = {
  // Place default values for settings here
  CONFIG_VERSION,
  2,
  { (TemperatureSensor) {
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
      { 0x28, 0xE3, 0xD7, 0x1D, 0x07, 0x00, 0x00, 0xBE },
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
      HIGH,
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
      1,
#endif
      "Boiler", LIQUID
    }, (TemperatureSensor) {
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
      { 0x28, 0x33, 0x47, 0x1E, 0x07, 0x00, 0x00, 0x45 },
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
      LOW,
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
      2,
#endif
      "Column", VAPOR
    }
  }
};


/*-----( Declare variables )-----*/
// Declare array of sensors
TemperatureSensor temperatureSensors[MAXIMUM_TEMPERATURE_SENSORS];

int configAddress;

//Declare asynchronous function last event times
unsigned long lastTemperatureRead;
unsigned long lastPressureRead;
unsigned long lastUserInput;
unsigned long lastDisplayWrite;

void setup() {

  Serial.begin(9600);

  // Initialise settings
  initSettings();
  // Initialise temperature sensors
  initTemperatureSensors();
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
  doFunctionAtInterval(readPressureSensors, &lastPressureRead, READ_PRESSURE_SENSORS_EVERY);  	// read pressure sensors
  doFunctionAtInterval(readUserInput, &lastUserInput, READ_USER_INPUT_EVERY);  // read user input
  doFunctionAtInterval(writeDisplay, &lastDisplayWrite, WRITE_DISPLAY_EVERY);  // write values to display

}

void initSettings() {

  Settings tempSettings;
  waitForEEPROM();
  EEPROM.setMemPool(memoryBase, EEPROMSizeMega);
  waitForEEPROM();
  configAddress = EEPROM.getAddress(sizeof(Settings));
  waitForEEPROM();
  EEPROM.readBlock(configAddress, tempSettings);
  waitForEEPROM();
  if (tempSettings.version != CONFIG_VERSION) {
    // Settings have not been saved before or settings configuration has changed
    EEPROM.writeBlock(configAddress, settings);
  }
  waitForEEPROM();
  EEPROM.readBlock(configAddress, settings);
  waitForEEPROM();
}

void updateSettings() {
  waitForEEPROM();
  EEPROM.updateBlock(configAddress, settings);
  waitForEEPROM();

}

void initTemperatureSensors() {

  for (int i = 0; i < settings.sensorsUsed; i++) {
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
    memcpy(temperatureSensors[i].address, settings.sensors[i].address, sizeof(DeviceAddress) / sizeof(uint8_t));
<<<<<<< HEAD
=======
    //memcpy(temperatureSensors[i].coefficients, settings.sensors[i].coefficients, 5);
>>>>>>> refs/remotes/origin/master
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
    temperatureSensors[i].select172 = settings.sensors[i].select172;
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
    temperatureSensors[i].rtdChannel = settings.sensors[i].rtdChannel;
#endif
    temperatureSensors[i].sensorName = settings.sensors[i].sensorName;
    temperatureSensors[i].state = settings.sensors[i].state;
  }

  // Initialise temperature sensors
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
  sensors.begin();
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR

#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
  I2C_RTD_PORTNAME.begin();
  rtds.Disable_All_RTD_Channels();							// Disable all RTD channels
  rtds.Set_RTD_SPS(RTD_SAMPLE_FREQUENCY);						// Slow the shield down
#endif

  for (int i = 0; i < MAXIMUM_TEMPERATURE_SENSORS; i++) {
    // Initialise looping/repeated temperature sensor values

#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
    //Set temperature resolution
    sensors.setResolution(temperatureSensors[i].address, TEMPERATURE_PRECISION);
#endif
#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
    // set port 15 as output and low so that the top sensor is connected to pin 4
    pinMode(SMT172_SELECT, OUTPUT);
    digitalWrite(SMT172_SELECT, LOW);
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
    rtds.Enable_RTD_Channel(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel);                // Enable the RTD channel for each sensor
    rtds.Set_RTD_Idac(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel, RTD_DRIVE_CURRENT);	// Set the RTD drive current for each sensor
    rtds.Set_RTD_PGA(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel, RTD_PGA);              // Set the PGA value for each sensor
#endif

    // set tToABV object to calculate correct ABV values
    switch (temperatureSensors[i].state) {
      case VAPOR:
        temperatureSensors[i].tToABV.Vapor();
        break;
      case LIQUID:
        temperatureSensors[i].tToABV.Liquid();
        break;
      default:
        // something is wrong, set to vapor
        temperatureSensors[i].tToABV.Vapor();
        break;
    }
  }
  delay(2000);	// A short delay so that the first reading can be taken.
  readTemperatureSensors();

}

void readTemperatureSensors() {

  // Read temperature sensor values
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR
  sensors.requestTemperatures();
#endif

<<<<<<< HEAD
=======
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR

#endif

>>>>>>> refs/remotes/origin/master
  for (int i = 0; i < settings.sensorsUsed; i++) {
#ifdef FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR

    switch (temperatureSensors[i].state) {
      case LIQUID:
        {
          float coefficients[] = { -0.139408013, -0.006497086, 0.000239295, -284829E-06, 2.26093E-08 };
          temperatureSensors[i].tToABV.Temperature(sensors.getTempC(temperatureSensors[i].address) - calculatePolynomial(coefficients, sensors.getTempC(temperatureSensors[i].address)));
          break;
        }
      case VAPOR:
        {
          float coefficients[] = { 0, 0, 0, 0, 0 };
          temperatureSensors[i].tToABV.Temperature(sensors.getTempC(temperatureSensors[i].address) - calculatePolynomial(coefficients, sensors.getTempC(temperatureSensors[i].address)));
          break;
        }
    }
#endif

#ifdef FEATURE_ENABLED_SMT172_TEMPERATURE_SENSOR
    digitalWrite(SMT172_SELECT, temperatureSensors[i].select172);
    SMT172::startTemperature(0.002);
repeatCase0:
    switch (SMT172::getStatus()) {
      case BUSY: goto repeatCase0; // O Dijkstra, be merciful onto me, for I have sinned against you :) and so have I Edwin! :D wiifm
      case SUCCESS:
        temperatureSensors[i].tToABV.Temperature(SMT172::getTemperature());
        break;
      case NOT_CONNECTED: // Do nothing for now
        break;
    }
#endif
#ifdef FEATURE_ENABLED_PROTOVOLTAICS_PT100_TEMPERATURE_SENSOR
    temperatureSensors[i].tToABV.Temperature(rtds.Get_RTD_Temperature_degC(RTD_SENSOR_WIRES, temperatureSensors[i].rtdChannel));
#endif
  }

}

float calculatePolynomial(float* coefficients, float T) {

  float calculatedValue = 0;

  for (int i = 0; i < 5; i++) {
    calculatedValue += coefficients[i] * pow(T, i);
  }
  return calculatedValue;

}

void initPressureSensors() {

  // Set pressure for each temperature sensor to default pressure
  for (int i = 0; i < MAXIMUM_TEMPERATURE_SENSORS; i++) {
    temperatureSensors[i].tToABV.Pressure(defaultPressure);
  }

#if (defined(FEATURE_ENABLED_ADAFRUIT_BMP180) || defined(FEATURE_ENABLED_SPARKFUN_BMP180))
  if (!bmp.begin()) {
    // There was a problem detecting the BMP180 ... check your connections
    serialDivider();
    Serial.println("No BMP180 detected ... Check your wiring or I2C ADDR!");
    serialDivider();
  } else {
    readPressureSensors();
  }

#endif

}

void readPressureSensors() {

#ifdef FEATURE_ENABLED_ADAFRUIT_BMP180
  sensors_event_t event;

  bmp.getEvent(&event);
  if (event.pressure) {
    for (int i = 0; i < MAXIMUM_TEMPERATURE_SENSORS; i++) {
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
          for (int i = 0; i < MAXIMUM_TEMPERATURE_SENSORS; i++) {
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

  for (int i = 0; i < settings.sensorsUsed; i++) {
    Serial.println(temperatureSensors[i].sensorName);
    Serial.print("Temperature: "); Serial.print(temperatureSensors[i].tToABV.Temperature(), 2); Serial.println("c");
    Serial.print("Pressure: "); Serial.print(temperatureSensors[i].tToABV.Pressure(), 2);  Serial.println("hPa");
    Serial.print("ABV: "); Serial.print(temperatureSensors[i].tToABV.ABV(), 2); Serial.println("%");
    serialDivider();
  }

}

void serialDivider() {

  Serial.println("---------------------------------------");

}

void waitForEEPROM() {
  while (!EEPROM.isReady()) {
    delay(1);
  }
}

