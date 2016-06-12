# EParrotFrameworkSketch

Framework Arduino sketch for Vision Stills e-Parrot projects

## Explanation
A framework sketch to begin e-Parrot Arduino projects.
### Hardware Currently Supported

#### Pressure Sensors
##### No pressure sensor
Pressure defaults to 1013.25hPa
##### BMP180 pressure sensor
Support for Sparkfun BMP180 Library or Adafruit BMP180 Library

#### Temperature Sensors
##### Maxim Integrated DS18B20
##### SMT172 Sensor
##### Protovoltaics Shield with Pt-100 Sensors

#### Displays
##### Serial Display

### Framework for EEPROM Storage of Settings

## Install

## Usage
To enable a feature, uncomment the relevant #define FEATURE_ENABLED_ at the top of the sketch. e.g. To enable a BMP180 pressure sensor with the Adafruit library, change line 2 from
```c++
//#define FEATURE_ENABLED_ADAFRUIT_BMP180
```
to
```c++
#define FEATURE_ENABLED_ADAFRUIT_BMP180
```

## License
See license [here](../master/LICENSE)
