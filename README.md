# arduino-modbus-i2c-bridge

Arduino bridge to present I2C sensor values on a Modbus network.

Tested with Arduino Pro Mini (328p) but other boards should have success.

## Features

### Supported sensors:

* SHT31 - Humidity and temperature sensor.
* SGP30 - Multi-gas (VOC and CO₂eq) sensor.

### General

* Uses software serial and I2C libraries so any hardware pins can be used (search '_PIN' in code to change).
* Instantaneous and moving average (configurable period) for all values.
* Delta (relative change over period) values for humidity/temperature (configurable period).
* Automatic detection of connected sensors.
* Automatic disabling of a failed sensor (read of input registers for failed/missing sensor return Modbus exception). Reboot/power cycle microcontroller after replacing failed sensors.

### Sensor specific

* SGP30 baseline is periodically saved in EEPROM and used to initialise same device (serial number check) on power up.
* SGP30 humidity compensation occurs when SHT31 sensor is also available.


## Holding Registers

Updating any of these writeable registers causes an instant 'restart' (re-initialisation).

| #   | Use                             | R/W | Native Range | Datatype            | Default      |
| --- | ------------------------------- | --- | ------------ | ------------------- | ------------ |
| 0   | Software Version                | R   |              | 16 bit unsigned int | *Version*    |
| 1   | Device address                  | R/W | 1 - 247      | 16 bit unsigned int | 1            |
| 2   | Baud rate                       | R/W | 2400 - 57600 | 16 bit unsigned int | 9600         |
| 3   | SHT31 polling Frequency (ms)    | R/W | 0 - 65535    | 16 bit unsigned int | 1000         |
| 4   | SHT31 moving average period (s) | R/W | 0 - 65535    | 16 bit unsigned int | 30           |
| 5   | SHT31 delta period (s)          | R/W | 0 - 65535    | 16 bit unsigned int | 30           |
| 6   | SGP30 polling Frequency (ms)    | R/W | 0 - 65535    | 16 bit unsigned int | 1000         |
| 7   | SGP30 moving average period (s) | R/W | 0 - 65535    | 16 bit unsigned int | 120          |
| 8   | SGP30 Serial #1                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 9   | SGP30 Serial #2                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 10  | SGP30 Serial #3                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 11  | SGP30 eCO2 Baseline             | R   |              | 16 bit unsigned int | *From SGP30* |
| 12  | SGP30 TVOC Baseline             | R   |              | 16 bit unsigned int | *From SGP30* |

## Input Registers

| #   | Device | Reading                      | Native Range | Datatype            | Scale |
| --- | ------ | ---------------------------- | ------------ | ------------------- | ----- |
| 0   | SHT31  | Temperature                  | -40 - 125°C  | 16 bit signed int   | x 100 |
| 1   | SHT31  | Humidity                     | 0 - 100%     | 16 bit unsigned int | x 100 |
| 2   | SHT31  | Temperature (moving average) | -40 - 125°C  | 16 bit signed int   | x 100 |
| 3   | SHT31  | Humidity (moving average)    | 0 - 100%     | 16 bit unsigned int | x 100 |
| 4   | SHT31  | Temperature delta            | -165 - 165°C | 16 bit signed int   | x 100 |
| 5   | SHT31  | Humidity delta               | -100 - 100%  | 16 bit signed int   | x 100 |
| 6   | SGP30  | Raw H2                       |              | 16 bit unsigned int |       |
| 7   | SGP30  | Raw ethanol                  |              | 16 bit unsigned int |       |
| 8   | SGP30  | eCO2                         | 0 - 1000ppm  | 16 bit unsigned int |       |
| 9   | SGP30  | TVOC                         | 0 - 1000ppb  | 16 bit unsigned int |       |
| 10  | SGP30  | IAQ index                    |              | 16 bit unsigned int |       |
| 11  | SGP30  | eCO2 (moving average)        | 0 - 1000ppm  | 16 bit unsigned int |       |
| 12  | SGP30  | TVOC (moving average)        | 0 - 1000ppb  | 16 bit unsigned int |       |
| 13  | SGP30  | IAQ index (moving average)   |              | 16 bit unsigned int |       |

## Hardware connections

Default pin configurations is:

* Modbus RX -> pin 16 (A2)
* Modbus TX -> pin 17 (A3)
* I2C Clock -> pin 15 (A1)
* I2C Data -> pin 14 (A0)

## Example configuration

**Note:** Examples assume Linux `modpoll` with USB RS485 on `/dev/ttyUSB0`.

Default Modbus address is 1. Even without sensors connected the device should respond to read all holding registers:

`modpoll -m rtu -b 9600 -p none -a 1 -0 -1 -t 4 -r 0 -c 13 /dev/ttyUSB0`

To change Modbus address from 1 to 2:

`modpoll -m rtu -b 9600 -p none -a 1 -0 -1 -t 4 -r 1 -c 1 /dev/ttyUSB0 2`

To read all input registers (must have all sensors connected or a Modbus exception will be returned):

`modpoll -m rtu -b 9600 -p none -a 1 -0 -1 -t 3 -r 0 -c 14 /dev/ttyUSB0`

## References/Credits

Some of this shamelessly copied from Adafruit libraries:

* https://github.com/adafruit/Adafruit_SHT31
* https://github.com/adafruit/Adafruit_SGP30

