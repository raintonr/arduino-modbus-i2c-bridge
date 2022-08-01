//  References:
//
// - ModbusRTUSlave Library
// - Adafruit SHT31
// - Adafruit SGP30

#define DEBUG

//////////////////////////////////////////////////////////////////////////////
// Misc stuff

// Flash status LED to indicate feedback, errors, etc.
void status_flash(uint16_t ms) {
  unsigned long stopFlash = millis() + ms;
  while (stopFlash > millis()) {
    digitalWrite(LED_BUILTIN, (millis() % 40 < 10) ? 1 : 0);
  }
  digitalWrite(LED_BUILTIN, 0);
}

// Decode uint16_t from 2 byte buffer
uint16_t decode_uint16_t(uint8_t readbuffer[2]) {
  uint16_t out = readbuffer[0];
  out <<= 8;
  out |= readbuffer[1];
  return out;
}

// Logarithmic regression calculator...
class LogarithmicRegressionCalculator {
 public:
  LogarithmicRegressionCalculator(float add, float mult);
  uint16_t calc(uint16_t x);

 private:
  float add, mult;
};

LogarithmicRegressionCalculator::LogarithmicRegressionCalculator(float add,
                                                                 float mult) {
  this->add = add;
  this->mult = mult;
}

uint16_t LogarithmicRegressionCalculator::calc(uint16_t x) {
  float y = 0;
  if (x > 0) {
    y = log(x);
    y *= mult;
    y += add;
    if (y < 0) y = 0;
  }
  return y;
}

// Moving average calculator to smooth readings.
// This is all done in integer (long) arithmatic so no rounding errors (we
// hope!).
class MovingAverageCalculator {
 public:
  MovingAverageCalculator(int readings);
  void sample(int input);
  int current_average();
  int current_sample();

 private:
  void MovingAverageCalculator::new_reading(int input);
  long total;
  int last_sample;
  int num_readings;
  boolean init;
};

MovingAverageCalculator::MovingAverageCalculator(int readings) {
  this->total = 0;
  this->num_readings = readings;
  this->init = false;
}
void MovingAverageCalculator::sample(int input) {
  if (!init) {
    init = true;
    this->total = input;
    this->total *= this->num_readings;
  } else {
    // Update running total by subtracting the current average & replacing with
    // that passed in.

    this->total -= this->current_average();
    this->total += input;
  }
  this->last_sample = input;
}

// Return current moving average
int MovingAverageCalculator::current_average() {
  return this->total / this->num_readings;
}

// Return last value sampled
int MovingAverageCalculator::current_sample() { eturn this->last_sample; }

//////////////////////////////////////////////////////////////////////////////
// To calculate an Indoor Air Quality (IAQ) index from 0-500...
//
//      TVOC (ppb): 100 333 1000 3333 8332
// Maps to our IAQ: 10 100 200 300 400
//
//       CO2 (ppm): 400 600 1000 1500 2500
// Maps to our IAQ: 10 100 200 300 400
//
// This is basically the bands from Awair:
//
// https://support.getawair.com/hc/en-us/articles/360039242373-Air-Quality-Factors-Measured-By-Awair-Element
//
// Use a functional approximation calculator to convert the above tables
// into approximation for our IAQ. This yields:
// - TVOC IAQ = -402.3294+87.6842*ln(x)
// - CO2 IAQ = -1269.4589+213.6673*ln(x)
//

LogarithmicRegressionCalculator *LR_TVOC =
    new LogarithmicRegressionCalculator(-402.3294, 87.6842);
LogarithmicRegressionCalculator *LR_CO2 =
    new LogarithmicRegressionCalculator(-1269.4589, 213.6673);

//////////////////////////////////////////////////////////////////////////////
// I2C Stuff

// We use BitBang here for ease of build as hardware I2C pins are not on the
// edge of a Pro Mini board.

#include <BitBang_I2C.h>
BBI2C bbi2c;
#define I2C_SDA_PIN 14
#define I2C_SCL_PIN 15
#define I2C_CLOCK_FREQUENCY 200000

void i2C_setup() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.iSDA = I2C_SDA_PIN;
  bbi2c.iSCL = I2C_SCL_PIN;
  I2CInit(&bbi2c, I2C_CLOCK_FREQUENCY);
  status_flash(100);  // allow devices to power up
}

bool check_i2c_write(uint8_t iAddr, uint8_t *pData, int iLen,
                     int16_t write_delay) {
  // Assume failure
  bool ret = false;

  // Turn on LED to indicate I2C traffic
  digitalWrite(LED_BUILTIN, 1);

  if (I2CWrite(&bbi2c, iAddr, pData, iLen) != iLen) {
    // Error
#ifdef DEBUG
    Serial.print("Failed to write ");
    Serial.print(iLen);
    Serial.print(" bytes to ");
    Serial.println(iAddr);
#endif
    // Flash for 2s (turns off when done)
    status_flash(2000);
  } else {
    ret = true;
    delay(write_delay);
    // Don't forget to turn LED off
    digitalWrite(LED_BUILTIN, 0);
  };

  return ret;
}

static uint8_t crc8(const uint8_t *data, int len) {
  /*

     CRC-8 formula from page 14 of SHT spec pdf

     Test data 0xBE, 0xEF should yield 0x92

     Initialization data 0xFF
     Polynomial 0x31 (x8 + x5 +x4 +1)
     Final XOR 0x00
  */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

//////////////////////////////////////////////////////////////////////////////
// SGP30 Stuff

// After the “sgp30_iaq_init” command, a “sgp30_measure_iaq” command has to be
// sent in regular intervals of 1s to ensure proper operation of the dynamic
// baseline compensation algorithm.
// So this is a constant & cannot be changed.
#define SGP30_MEASURE_IAQ_PERIOD 1000

unsigned long sgp30_next_measure_iaq;
unsigned long sgp30_next_get_iaq_baseline;

// SGP30 sensor can return 'spikey' readings so 2-3 minutes is a good moving
// average period to smooth them. So let's use 2 minutes.

#define SGP30_MA_SAMPLES 120000 / SGP30_MEASURE_IAQ_PERIOD

MovingAverageCalculator *sgp30_eco2_ma =
    new MovingAverageCalculator(SGP30_MA_SAMPLES);
MovingAverageCalculator *sgp30_tvoc_ma =
    new MovingAverageCalculator(SGP30_MA_SAMPLES);

uint16_t sgp30_iaq_baseline_eco2;
uint16_t sgp30_iaq_baseline_tvoc;

// Period to get/save baseline - 12 hours
#define SGP30_GET_IAQ_BASELINE_PERIOD 43200000

#define SGP30_I2C_ADDRESS 0x58
bool have_sgp30 = false;

uint8_t SGP30_READSERIAL[] = {0x36, 0x82};
uint8_t SGP30_IAQ_INIT[] = {0x20, 0x03};
uint8_t SGP30_MEASURE_IAQ[] = {0x20, 0x08};
uint8_t SGP30_GET_IAQ_BASELINE[] = {0x20, 0x15};

void sgp30_setup() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  // Check we have this device
  if (I2CTest(&bbi2c, SGP30_I2C_ADDRESS)) {
#ifdef DEBUG
    Serial.println("Found SGP30");
#endif
    have_sgp30 = true;

    sgp30_serial();

    // TODO: compare serial with flash & read baseline if one set and it
    // matches.

    sgp30_iaq_init();

    // Read right away
    sgp30_next_measure_iaq = millis();

    // But wait one period before getting baseline
    sgp30_next_get_iaq_baseline = millis() + SGP30_GET_IAQ_BASELINE_PERIOD;
  } else {
#ifdef DEBUG
    Serial.println("SGP30 not found");
#endif
  }
}

void sgp30_loop() {
  if (!have_sgp30) return;

  // TODO: set absolute humidity from SHT31 if available.

  if (millis() >= sgp30_next_measure_iaq) {
    sgp30_next_measure_iaq += SGP30_MEASURE_IAQ_PERIOD;
    sgp30_measure_iaq();
  }
  if (millis() >= sgp30_next_get_iaq_baseline) {
    sgp30_next_get_iaq_baseline += SGP30_GET_IAQ_BASELINE_PERIOD;
    sgp30_get_iaq_baseline();
  }
}

void sgp30_serial() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  if (check_i2c_write(SGP30_I2C_ADDRESS, SGP30_READSERIAL,
                      sizeof(SGP30_READSERIAL), 10)) {
    // The get serial ID command returns 3 words, and every word is followed by
    // an 8-bit CRC checksum. Together the 3 words constitute a unique serial ID
    // with a length of 48 bits.

    uint8_t readbuffer[9];
    if (I2CRead(&bbi2c, SGP30_I2C_ADDRESS, readbuffer, sizeof(readbuffer))) {
#ifdef DEBUG
      Serial.print("Read serial: ");
      for (int lp = 0; lp < sizeof(readbuffer); lp++) {
        if (lp > 0) {
          Serial.print(":");
        }
        Serial.print(readbuffer[lp], HEX);
      }
      Serial.print("\n");
#endif

      // Check CRCs...
      if (crc8(&readbuffer[0], 2) != readbuffer[2] ||
          crc8(&readbuffer[3], 2) != readbuffer[5] ||
          crc8(&readbuffer[6], 2) != readbuffer[8]) {
#ifdef DEBUG
        Serial.println("Bad CRC");
#endif
        status_flash(1000);
      } else {
        // Copy/store serial if changed...
      }
    }
  }
}

// Initialise SGP30
// A new “sgp30_iaq_init” command has to be sent after every power-up or soft
// reset.

void sgp30_iaq_init(void) {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  check_i2c_write(SGP30_I2C_ADDRESS, SGP30_IAQ_INIT, sizeof(SGP30_IAQ_INIT),
                  10);
}

// Get baseline

void sgp30_get_iaq_baseline() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  if (check_i2c_write(SGP30_I2C_ADDRESS, SGP30_GET_IAQ_BASELINE,
                      sizeof(SGP30_GET_IAQ_BASELINE), 10)) {
    uint8_t readbuffer[6];
    if (I2CRead(&bbi2c, SGP30_I2C_ADDRESS, readbuffer, sizeof(readbuffer))) {
      // Check CRCs...
      if (crc8(&readbuffer[0], 2) != readbuffer[2]) {
#ifdef DEBUG
        Serial.println("Bad eCO2 baseline CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_iaq_baseline_eco2 = decode_uint16_t(&readbuffer[0]);
      }

      if (crc8(&readbuffer[3], 2) != readbuffer[5]) {
#ifdef DEBUG
        Serial.println("Bad TVOC baseline CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_iaq_baseline_tvoc = decode_uint16_t(&readbuffer[3]);
      }

#ifdef DEBUG
      Serial.print("Read SGP30 baseline: ");
      Serial.print(sgp30_iaq_baseline_eco2);
      Serial.print("\t");
      Serial.println(sgp30_iaq_baseline_tvoc);
#endif
    }
  }
}

void sgp30_set_iaq_baseline(uint16_t baseline_eco2, uint16_t baseline_tvoc) {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  // Yes, the eco2/TVOC order is reversed in this set command.

  uint8_t writebuffer[8];
  writebuffer[0] = 0x20;
  writebuffer[1] = 0x1e;
  writebuffer[2] = baseline_tvoc >> 8;
  writebuffer[3] = baseline_tvoc & 0xFF;
  writebuffer[4] = crc8(&writebuffer[2], 2);
  writebuffer[5] = baseline_eco2 >> 8;
  writebuffer[6] = baseline_eco2 & 0xFF;
  writebuffer[7] = crc8(&writebuffer[5], 2);

  check_i2c_write(SGP30_I2C_ADDRESS, writebuffer, sizeof(writebuffer), 10);
}

// Read IAQ values and calculate our index

void sgp30_measure_iaq() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  // It is recommended (well, possible) to send sgp30_set_absolute_humidity
  // command before each reading, but this requires *absolute* humidity and the
  // SHT31 used here cannot supply that.

  if (check_i2c_write(SGP30_I2C_ADDRESS, SGP30_MEASURE_IAQ,
                      sizeof(SGP30_MEASURE_IAQ), 12)) {
    uint8_t readbuffer[6];
    if (I2CRead(&bbi2c, SGP30_I2C_ADDRESS, readbuffer, sizeof(readbuffer))) {
      // Check CRCs...
      if (crc8(&readbuffer[0], 2) != readbuffer[2]) {
#ifdef DEBUG
        Serial.println("Bad eCO2 CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_eco2_ma->sample(decode_uint16_t(&readbuffer[0]));
      }

      if (crc8(&readbuffer[3], 2) != readbuffer[5]) {
#ifdef DEBUG
        Serial.println("Bad TVOC CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_tvoc_ma->sample(decode_uint16_t(&readbuffer[3]));
      }

#ifdef DEBUG
      Serial.print("Read SGP30:\t");
      Serial.print(sgp30_eco2_ma->current_sample());
      Serial.print("\t");
      Serial.print(sgp30_tvoc_ma->current_sample());
      Serial.print("\tMAs:\t");
      Serial.print(sgp30_eco2_ma->current_average());
      Serial.print("\t");
      Serial.print(sgp30_tvoc_ma->current_average());
      Serial.print("\n");
#endif
    }
  }
}

// Shamelessly copied from Adafruit examples... ahem... ;)
// return absolute humidity [mg/m^3] with approximation formula
// @param temperature [°C]
// @param humidity [%RH]

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity =
      216.7f * ((humidity / 100.0f) * 6.112f *
                exp((17.62f * temperature) / (243.12f + temperature)) /
                (273.15f + temperature));  // [g/m^3]
  const uint32_t absoluteHumidityScaled =
      static_cast<uint32_t>(1000.0f * absoluteHumidity);  // [mg/m^3]
  return absoluteHumidityScaled;
}

// Calculate our IAQ
int sgp30_calc_iaq(int eco2, int tvoc) {
  // Calculate both and use the highest one.
  int iaq_eco2 = LR_CO2->calc(eco2);
  int iaq_tvoc = LR_TVOC->calc(tvoc);

#ifdef DEBUG
  Serial.print("IAQ CO2: ");
  Serial.print(iaq_eco2);
  Serial.print("\tIAQ TVOC: ");
  Serial.println(iaq_tvoc);
#endif
  return iaq_eco2 > iaq_tvoc ? iaq_eco2 : iaq_tvoc;
}

//////////////////////////////////////////////////////////////////////////////
// SHT31 Stuff

#define SHT31_I2C_ADDRESS 0x44
bool have_sht31 = false;

unsigned long sht31_nextRead;
const unsigned long sht31_period = 5000;  // TODO: should come from register.

// 30 second moving average for temperature/humidity. TODO: should come from
// register.
#define SHT31_MA_SAMPLES 30000 / sht31_period

MovingAverageCalculator *sht31_temperature_ma =
    new MovingAverageCalculator(SHT31_MA_SAMPLES);
MovingAverageCalculator *sht31_humidity_ma =
    new MovingAverageCalculator(SHT31_MA_SAMPLES);

void sht31_setup() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  // Check we have this device
  if (I2CTest(&bbi2c, SHT31_I2C_ADDRESS)) {
#ifdef DEBUG
    Serial.println("Found SHT31");
#endif
    have_sht31 = true;

    // Read SHT31 right away
    sht31_nextRead = millis();
  } else {
#ifdef DEBUG
    Serial.println("SHT31 not found");
#endif
  }
}

void sht31_loop() {
  if (!have_sht31) return;
  if (millis() >= sht31_nextRead) {
    sht31_nextRead += sht31_period;
    sht31_read();
  }
}

// Stolen from Adafruit SHT31 code

uint8_t readbuffer[6];
uint8_t SHT31_MEAS_HIGHREP[] = {0x24, 0x00};
uint8_t SHT31_READSTATUS[] = {0xF3, 0x2D};
uint8_t SHT31_HEATEREN[] = {0x30, 0x6D};  /* Heater Enable */
uint8_t SHT31_HEATERDIS[] = {0x30, 0x66}; /* Heater Disable */
#define SHT31_REG_MSB_HEATER_BITMASK 0x20 /* Status Register Heater Bit */

bool sht31_isHeaterEnabled() {
  if (check_i2c_write(SHT31_I2C_ADDRESS, SHT31_READSTATUS,
                      sizeof(SHT31_READSTATUS), 20)) {
    uint8_t readbuffer[3];
    if (I2CRead(&bbi2c, SHT31_I2C_ADDRESS, readbuffer, sizeof(readbuffer))) {
#ifdef DEBUG
      Serial.print("Read status: ");
      Serial.print(readbuffer[0], HEX);
      Serial.print("/");
      Serial.println(readbuffer[1], HEX);
#endif
      if (readbuffer[2] != crc8(readbuffer, 2)) {
#ifdef DEBUG
        Serial.println("Bad CRC");
#endif
        status_flash(1000);
        return 0;
      } else {
        return (readbuffer[0] & SHT31_REG_MSB_HEATER_BITMASK);
      }
    }
  };
}

void sht31_Heater(bool heater_on) {
  if (heater_on) {
    check_i2c_write(SHT31_I2C_ADDRESS, SHT31_HEATEREN, sizeof(SHT31_HEATEREN),
                    10);
  } else {
    check_i2c_write(SHT31_I2C_ADDRESS, SHT31_HEATERDIS, sizeof(SHT31_HEATERDIS),
                    10);
  }
}

void sht31_read() {
  // Make sure heater is always off
  if (sht31_isHeaterEnabled()) {
#ifdef DEBUG
    Serial.println("SHT31 heater is on - turning off");
#endif
    sht31_Heater(false);
    // And wait for 30s
    status_flash(30000);
  }

  /* Measurement High Repeatability with Clock Stretch Disabled */
  if (check_i2c_write(SHT31_I2C_ADDRESS, SHT31_MEAS_HIGHREP,
                      sizeof(SHT31_MEAS_HIGHREP), 20)) {
    if (I2CRead(&bbi2c, SHT31_I2C_ADDRESS, readbuffer, sizeof(readbuffer))) {
      // Stolen from Adafruit library
      if (readbuffer[2] != crc8(readbuffer, 2) ||
          readbuffer[5] != crc8(readbuffer + 3, 2)) {
#ifdef DEBUG
        Serial.println("Bad SHT31 CRC");
#endif
        status_flash(500);
      } else {
        // Calculations take from Adafruit library
        int32_t stemp =
            (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
        // simplified (65536 instead of 65535) integer version of:
        // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
        stemp = ((4375 * stemp) >> 14) - 4500;
        // Test negative temperatures
        // stemp -= 5000;
        sht31_temperature_ma->sample(stemp);

        uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
        // simplified (65536 instead of 65535) integer version of:
        // humidity = (shum * 100.0f) / 65535.0f;
        shum = (625 * shum) >> 12;
        sht31_humidity_ma->sample(shum);

#ifdef DEBUG
        Serial.print("Read SHT31:\t");
        Serial.print(stemp);
        Serial.print("\t");
        Serial.print(shum);
        Serial.print("\tMAs:\t");
        Serial.print(sht31_temperature_ma->current_average());
        Serial.print("\t");
        Serial.print(sht31_humidity_ma->current_average());
        Serial.print("\n");
#endif
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Modbus stuff

// ModbusRTUSlave is not dependant on SoftwareSerial.
// It is included here so that the Serial port can be kept free for debugging.

#include <ModbusRTUSlave.h>
#include <SoftwareSerial.h>

#define modbusRxPin 16
#define modbusTxPin 17
#define modbusId 1                // TODO: should be register
const unsigned long baud = 9600;  // TODO: should be register
#define MODBUS_INPUT_REGISTERS 25
const word bufSize = 256;

// This is the buffer for the ModbusRTUSlave object.
// It is used to store the Modbus messages.
// A size of 256 bytes is recommended, but sizes as low as 8 bytes can be
// used.
byte buf[bufSize];

// Initilize a SoftwareSerial port.
SoftwareSerial mySerial(modbusRxPin, modbusTxPin);

// Initilize a ModbusRTUSlave.
ModbusRTUSlave modbus(mySerial, buf, bufSize);

void modbus_setup() {
  // Setup the SoftwareSerial port.
  mySerial.begin(baud);

  // Setup the ModbusRTUSlave
  modbus.begin(modbusId, baud);

  // Configure the inputRegister(s).
  modbus.configureInputRegisters(MODBUS_INPUT_REGISTERS, inputRegisterRead);
}

// This is a funciton that will be passed to the ModbusRTUSlave for reading
// input registers.
long inputRegisterRead(word address) {
#ifdef DEBUG
  Serial.print("inputRegisterRead : ");
  Serial.println(address);
#endif

  // TODO: Clean up?
  switch (address) {
    case 10:
      return (uint16_t)sht31_temperature_ma->current_sample();
    case 11:
      return sht31_humidity_ma->current_sample();
    case 12:
      return (uint16_t)sht31_temperature_ma->current_average();
    case 13:
      return sht31_humidity_ma->current_average();
    case 22:
      return sgp30_eco2_ma->current_sample();
    case 23:
      return sgp30_tvoc_ma->current_sample();
    case 24:
      return sgp30_calc_iaq(sgp30_eco2_ma->current_sample(),
                            sgp30_tvoc_ma->current_sample());
    case 25:
      return sgp30_eco2_ma->current_average();
    case 26:
      return sgp30_tvoc_ma->current_average();
    case 27:
      return sgp30_calc_iaq(sgp30_eco2_ma->current_average(),
                            sgp30_tvoc_ma->current_average());
      ;
  }
#ifdef DEBUG
  Serial.println("Nothing!");
#endif
  return NAN;
}

//////////////////////////////////////////////////////////////////////////////
// Setup

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Modbus SHT31 sensor");
#endif

  i2C_setup();
  sht31_setup();
  sgp30_setup();
  modbus_setup();
}

//////////////////////////////////////////////////////////////////////////////
// Main loop

void loop() {
  // Poll for Modbus RTU requests from the master device.
  // This will autmatically run the inputRegisterRead function as needed.
  modbus.poll();

  // Periodically read I2C stuff...

  sht31_loop();
  sgp30_loop();
}
