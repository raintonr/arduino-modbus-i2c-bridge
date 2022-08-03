# arduino-modbus-i2c-bridge
Arduino bridge to present I2C sensor values on a Modbus network

## Holding Registers

| #  | Use                             | R/W | Native Range | Datatype            | Default      |
| -- |-------------------------------- | --- | ------------ | ------------------- | ------------ |
| 0  | Software Version                | R   |              | 16 bit unsigned int | *Version*    |
| 1  | Device address                  | R/W | 1 - 247      | 16 bit unsigned int | 1            |
| 2  | Baud rate                       | R/W | 2400 - 57600 | 16 bit unsigned int | 9600         |
| 3  | SHT31 polling Frequency (ms)    | R/W | 0 - 65535    | 16 bit unsigned int | 1000         |
| 4  | SHT31 moving average period (s) | R/W | 0 - 65535    | 16 bit unsigned int | 30           |
| 5  | SHT31 delta period (s)          | R/W | 0 - 65535    | 16 bit unsigned int | 30           |
| 6  | SGP30 polling Frequency (ms)    | R/W | 0 - 65535    | 16 bit unsigned int | 1000         |
| 7  | SGP30 moving average period (s) | R/W | 0 - 65535    | 16 bit unsigned int | 120          |
| 8  | SGP30 Serial #1                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 9  | SGP30 Serial #2                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 10 | SGP30 Serial #3                 | R   |              | 16 bit unsigned int | *From SGP30* |
| 11 | SGP30 eCO2 Baseline             | R   |              | 16 bit unsigned int | *From SGP30* |
| 12 | SGP30 TVOC Baseline             | R   |              | 16 bit unsigned int | *From SGP30* |
