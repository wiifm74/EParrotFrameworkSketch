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
The idea behind this is that you turn on and off 'features'.

Let's assume:
[list]
[li]you are using the Arduino IDE (https://www.arduino.cc/en/Main/Software)[/li]
[li]you have a Mega2560 Board[/li]
[li]you have a BMP180 Pressure Sensor[/li]
[li]you have 2x DS18B20 Temperature Sensors and they are wired to one pin of the board using normal power mode (http://www.tweaking4all.com/wp-content/uploads/2014/03/ds18b20-normal-power.jpg)[/li]
[li]You know the addresses of your DS18B20 sensors.  If not, go here - [http://arduino-info.wikispaces.com/Brick-Temperature-DS18B20#Read%20individual[/li]
[li]Your board is connected to your computer via USB[/li]
[/list]

1. Open the sketch
- Run the Arduino IDE
- Click 'File' > 'Open' then select the EParrotFrameworkSketch.ino file e.g. 'C:\Users\wiifm\Documents\Arduino\EParrotFrameWorkSketch\EParrotFrameWorkSketch.ino'

2. Ensure IDE is communicating with your board
- Click 'Tools' > 'Board' select 'Arduino/Genuino Mega or Mega2560' (this is because we are using a Mega.  Choose YOUR board)
- Click 'Tools' > 'Port' and select the appropriate COM port

2. Enable Features
To enable a feature, uncomment the relevant #define FEATURE_ENABLED_ at the top of the sketch.
- To enable a BMP180 pressure sensor with the Adafruit library, change line 2 from
[code]//#define FEATURE_ENABLED_ADAFRUIT_BMP180[/code]
to
[code]#define FEATURE_ENABLED_ADAFRUIT_BMP180[/code]

- To enable the DS18B20 temp sensors, change line 4 from
[code]//#define FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR[/code]
to
[code]#define FEATURE_ENABLED_DS18B20_TEMPERATURE_SENSOR[/code]

3. Set DS18B20 sensor addresses
If you don't know your sensor addresses, go here - http://arduino-info.wikispaces.com/Brick-Temperature-DS18B20#Read%20individual
- Line 141 - Change the address values for your column sensor.  They are found DeviceAddress columnAddress = { HERE };
- Line 162 - Change the address values for your boiler sensor.  They are found DeviceAddress boilerAddress = { HERE };

4. Set the pin for DS18B20
- Click on the myBoard.ino tab
- change the number on line 5 to the correct pin
[code]#define ONE_WIRE_BUS 57[/code]
In this case, the data wire for your DS18B20 sensors is 57
- Click on the EParrotFrameWorkSketch tab

5. Save, compile and upload
- Click 'File' > 'Save'
- Click 'Sketch' > 'Verify/Compile' (or just click the big tick).  At this point, I am assuming you will see at the bottom of your screen a message saying 'Compiling sketch...' with a green bar then a message which says 'Done compiling.'
- Click 'Sketch' > 'Upload' (or just click the big right arrow).  You should see the message at the bottom say 'Compiling sketch...', 'Uploading...' then 'Done uploading.'

6. Check your results with serial monitor
- Click 'Tools' > 'Serial Monitor'
- Ensure you have '115200 baud' selected from the dropdown list in the bottom right hand corner of the serial monitor
- You should see values outputted to your screen in the format:

- Column
- Temperature: 0.00c
- Pressure: 1013.25hPa
- ABV: -7818.00%
-
- Boiler
- Temperature: 0.00c
- Pressure: 1013.25hPa
- ABV: -7818.00%



## License
See license [here](../master/LICENSE)
