//  References:
//
// - ModbusRTUSlave Library
// - Adafruit SHT31
// - Adafruit SGP30

#define SOFTWARE_VERSION 1

#undef DEBUG

//////////////////////////////////////////////////////////////////////////////
// Stuff we need to store in EEPROM with default values

#include <EEPROM-Storage.h>

// Start at zero and everything is 2 bytes + 1 check = 3 in total

EEPROMStorage<uint16_t> modbus_id(0, 1);
EEPROMStorage<uint16_t> modbus_baud_rate(3, 9600);
EEPROMStorage<uint16_t> sht31_period(6, 1000);
EEPROMStorage<uint16_t> sht31_ma_period(9, 30);
EEPROMStorage<uint16_t> sht31_delta_period(12, 30);
EEPROMStorage<uint16_t> sgp30_period(15, 1000);
EEPROMStorage<uint16_t> sgp30_ma_period(18, 120);

// Store each part of the serial individually. Bit clunky, but EEPROM-Storage
// does not support arrays.
EEPROMStorage<uint16_t> sgp30_serial_1(21, 0);
EEPROMStorage<uint16_t> sgp30_serial_2(24, 0);
EEPROMStorage<uint16_t> sgp30_serial_3(27, 0);

EEPROMStorage<uint16_t> sgp30_iaq_baseline_eco2(30, 0);
EEPROMStorage<uint16_t> sgp30_iaq_baseline_tvoc(33, 0);

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
uint16_t decode_uint16_t(uint8_t in[2]) {
  uint16_t out = in[0];
  out <<= 8;
  out |= in[1];
  return out;
}

//////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////
// Moving average calculator to smooth readings.
// This is all done in integer (long) arithmetic so no rounding errors (we
// hope!).
//
// This is a template because sometimes we want signed, sometimes unsigned data
// and to always store the correct type to save space (rather than using a long
// for example).

template <class statType>
class StatsCalc {
 public:
  void init_moving_average(uint16_t ma_period, uint16_t sample_period);
  void init_delta(uint16_t delta_period, uint16_t sample_period);
  void sample(statType input);
  statType current_sample();
  statType current_average();
  // Delta is always signed
  int16_t current_delta();

 private:
  statType last_sample;

  // Moving Average Stuff
  // Num readings are unsigned 16 bit because they come from Modbus registers.
  uint16_t ma_num_readings = 0;

  long ma_total;
  bool ma_init = false;

  // Delta stuff.
  // Period and samples are unsigned 16 bit because they come from Modbus
  // registers.
  uint16_t delta_period = 0;
  uint16_t delta_num_samples = 0;

  statType *delta_samples;
  uint16_t delta_current_slot;
  uint16_t delta_next_slot();
  bool delta_init = false;
};

template <typename statType>
void StatsCalc<statType>::init_moving_average(uint16_t ma_period,
                                              uint16_t sample_period) {
#ifdef DEBUG
  Serial.print("init_moving_average: ");
  Serial.print(ma_period);
  Serial.print("\t");
  Serial.print(sample_period);
  Serial.print("\t");
#endif

  // Delta period is in seconds, sample_period in ms
  int ma_num_readings = ma_period / (sample_period / 1000);
  // Always round up
  ma_num_readings++;

#ifdef DEBUG
  Serial.println(ma_num_readings);
#endif

  if (this->ma_init && ma_num_readings != this->ma_num_readings) {
#ifdef DEBUG
    Serial.println("Re-initialising ma...");
#endif
    this->ma_init = false;
  }

  this->ma_num_readings = ma_num_readings;
}

template <class statType>
void StatsCalc<statType>::init_delta(uint16_t delta_period,
                                     uint16_t sample_period) {
#ifdef DEBUG
  Serial.print("init_delta: ");
  Serial.print(delta_period);
  Serial.print("\t");
  Serial.print(sample_period);
  Serial.print("\t");
#endif

  // Delta period is in seconds, sample_period in ms
  int delta_num_samples = delta_period / (sample_period / 1000);
  // Always round up
  delta_num_samples++;

#ifdef DEBUG
  Serial.println(delta_num_samples);
#endif

  if (this->delta_init && (delta_period != this->delta_period ||
                           delta_num_samples != this->delta_num_samples)) {
#ifdef DEBUG
    Serial.println("Re-initialising delta...");
#endif
    free(this->delta_samples);
    this->delta_init = false;
  }
  this->delta_period = delta_period;
  this->delta_num_samples = delta_num_samples;
}

template <class statType>
void StatsCalc<statType>::sample(statType input) {
  if (this->ma_num_readings > 0) {
    // Calculate moving average
    if (!ma_init) {
#ifdef DEBUG
      Serial.println("ma_init");
#endif
      ma_init = true;
      this->ma_total = input;
      this->ma_total *= this->ma_num_readings;
    } else {
      // Update running total by subtracting the current average & replacing
      // with that passed in.

      this->ma_total -= this->current_average();
      this->ma_total += input;
    }
  }

  if (this->delta_num_samples > 0) {
    if (!this->delta_init) {
#ifdef DEBUG
      Serial.println("delta_init");
#endif
      this->delta_init = true;
      // Just fill all slots with current data
      int sample_buffer_size = this->delta_num_samples * sizeof(statType);
      this->delta_samples = (statType *)malloc(sample_buffer_size);
      if (this->delta_samples == NULL) {
        // Fatal error!
#ifdef DEBUG
        Serial.print("Could not malloc: ");
        Serial.println(sample_buffer_size);
#endif
        status_flash(-1);
      }

      for (int lp = 0; lp < this->delta_num_samples; lp++) {
        this->delta_samples[lp] = input;
      }
      this->delta_current_slot = 0;
    } else {
      this->delta_current_slot = this->delta_next_slot();
      this->delta_samples[this->delta_current_slot] = input;
    }
  }

  // Always stash last sample
  this->last_sample = input;
}

// Return current moving average
template <class statType>
statType StatsCalc<statType>::current_average() {
  return this->ma_total / this->ma_num_readings;
}

// Return last value sampled
template <class statType>
statType StatsCalc<statType>::current_sample() {
  return this->last_sample;
}

// Delta stuff
// The oldest slot *should* store the delta we're looking for so just use that
// for calculation.

template <class statType>
int16_t StatsCalc<statType>::current_delta() {
  return this->last_sample -
         this->delta_samples[this->delta_next_slot()];
}

template <class statType>
uint16_t StatsCalc<statType>::delta_next_slot() {
  uint16_t slot = this->delta_current_slot;
  return (++slot) % this->delta_num_samples;
}

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
#define I2C_BUFFER_SIZE 9

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

bool check_i2c_write(uint8_t iAddr, const uint8_t *pData, int iLen,
                     int16_t write_delay) {
  // Assume failure
  bool ret = false;

  // Turn on LED to indicate I2C traffic
  digitalWrite(LED_BUILTIN, 1);

  if (I2CWrite(&bbi2c, iAddr, (uint8_t *)pData, iLen) != iLen) {
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

// Shared global buffer
uint8_t i2c_buffer[I2C_BUFFER_SIZE];

bool check_i2c_response(uint8_t iAddr, const uint8_t *pData, int wLen,
                        int16_t write_delay, int rLen) {
  // Assume failure
  bool ret = false;

  if (check_i2c_write(iAddr, (uint8_t *)pData, wLen, write_delay)) {
    if (I2CRead(&bbi2c, iAddr, i2c_buffer, rLen)) {
      // We're good... unless CRC fails
      ret = true;
      // CRC is always done with 2 bytes data, 1 byte CRC
      for (int lp = 2; lp < rLen && ret; lp += 3) {
        if (crc8(&i2c_buffer[lp - 2], 2) != i2c_buffer[lp]) {
          ret = false;
#ifdef DEBUG
          Serial.println("Bad CRC");
#endif
          status_flash(500);
        }
      }
    }
  }

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
// So while that period is configurable via Modbus register it really
// shouldn't be changed.

unsigned long sgp30_next_measure_iaq;
unsigned long sgp30_next_get_iaq_baseline;

// Period to get/save baseline - 8 hours
#define SGP30_GET_IAQ_BASELINE_PERIOD 28800000

#define SGP30_I2C_ADDRESS 0x58
bool have_sgp30 = false;

const uint8_t SGP30_READSERIAL[] = {0x36, 0x82};
#define SGP30_READSERIAL_DELAY 10
#define SGP30_READSERIAL_RESPLEN 9
const uint8_t SGP30_IAQ_INIT[] = {0x20, 0x03};
const uint8_t SGP30_MEASURE_IAQ[] = {0x20, 0x08};
const uint8_t SGP30_MEASURE_RAW[] = {0x20, 0x50};
const uint8_t SGP30_GET_IAQ_BASELINE[] = {0x20, 0x15};

StatsCalc<uint16_t> *sgp30_eco2_stats = new StatsCalc<uint16_t>();
StatsCalc<uint16_t> *sgp30_tvoc_stats = new StatsCalc<uint16_t>();

void sgp30_init_stats() {
  sgp30_eco2_stats->init_moving_average(sgp30_ma_period, sgp30_period);
  sgp30_tvoc_stats->init_moving_average(sgp30_ma_period, sgp30_period);
}

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

    sgp30_init_stats();

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

  if (millis() >= sgp30_next_measure_iaq) {
    sgp30_next_measure_iaq += sgp30_period;
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

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_READSERIAL,
                         sizeof(SGP30_READSERIAL), SGP30_READSERIAL_DELAY,
                         SGP30_READSERIAL_RESPLEN)) {
    // The get serial ID command returns 3 words, and every word is followed
    // by an 8-bit CRC checksum. Together the 3 words constitute a unique
    // serial ID with a length of 48 bits.

    uint16_t s1 = decode_uint16_t(&i2c_buffer[0]);
    uint16_t s2 = decode_uint16_t(&i2c_buffer[3]);
    uint16_t s3 = decode_uint16_t(&i2c_buffer[6]);

#ifdef DEBUG_MORE
    Serial.print("Read serial: ");
    Serial.print(s1, HEX);
    Serial.print(":");
    Serial.print(s2, HEX);
    Serial.print(":");
    Serial.println(s3, HEX);
#endif

    // Check to see if this matches stored serial
    if (s1 != sgp30_serial_1 || s2 != sgp30_serial_2 || s3 != sgp30_serial_3) {
#ifdef DEBUG_MORE
      Serial.println("Storing new serial");
      sgp30_serial_1 = s1;
      sgp30_serial_2 = s2;
      sgp30_serial_3 = s3;
#endif
    } else {
      // Same sensor as last power up so get baseline readings from EEPROM
      // variables and write to sensor.
      if (sgp30_iaq_baseline_eco2 != 0 && sgp30_iaq_baseline_tvoc != 0) {
        sgp30_set_iaq_baseline(sgp30_iaq_baseline_eco2,
                               sgp30_iaq_baseline_tvoc);
      }
#ifdef DEBUG
      else {
        Serial.println("IAQ baseline is zero, ignoring");
      }
#endif
    }
  }
}

// Initialise SGP30
// A new “sgp30_iaq_init” command has to be sent after every power-up or soft
// reset.

void sgp30_iaq_init(void) {
  check_i2c_write(SGP30_I2C_ADDRESS, SGP30_IAQ_INIT, sizeof(SGP30_IAQ_INIT),
                  10);
}

// Get baseline

void sgp30_get_iaq_baseline() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_GET_IAQ_BASELINE,
                         sizeof(SGP30_GET_IAQ_BASELINE), 10, 6)) {
    sgp30_iaq_baseline_eco2 = decode_uint16_t(&i2c_buffer[0]);
    sgp30_iaq_baseline_tvoc = decode_uint16_t(&i2c_buffer[3]);

#ifdef DEBUG
    Serial.print("Read SGP30 baseline: ");
    Serial.print(sgp30_iaq_baseline_eco2);
    Serial.print("\t");
    Serial.println(sgp30_iaq_baseline_tvoc);
#endif
  }
}

void sgp30_set_iaq_baseline(uint16_t baseline_eco2, uint16_t baseline_tvoc) {
#ifdef DEBUG
  Serial.print("Setting IAQ baseline: ");
  Serial.print(baseline_eco2);
  Serial.print("\t");
  Serial.println(baseline_tvoc);
#endif

  // Yes, the eco2/TVOC order is reversed in this set command.

  i2c_buffer[0] = 0x20;
  i2c_buffer[1] = 0x1e;
  i2c_buffer[2] = baseline_tvoc >> 8;
  i2c_buffer[3] = baseline_tvoc & 0xFF;
  i2c_buffer[4] = crc8(&i2c_buffer[2], 2);
  i2c_buffer[5] = baseline_eco2 >> 8;
  i2c_buffer[6] = baseline_eco2 & 0xFF;
  i2c_buffer[7] = crc8(&i2c_buffer[5], 2);

  check_i2c_write(SGP30_I2C_ADDRESS, i2c_buffer, 8, 10);
}

// Read IAQ values and calculate our index

void sgp30_measure_iaq() {
#ifdef DEBUG
  Serial.println(__func__);
#endif

  // It is recommended  to send sgp30_set_absolute_humidity command before each
  // reading. We take care of that by setting after each SHT31 measurement.

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_MEASURE_IAQ,
                         sizeof(SGP30_MEASURE_IAQ), 12, 6)) {
    sgp30_eco2_stats->sample(decode_uint16_t(&i2c_buffer[0]));
    sgp30_tvoc_stats->sample(decode_uint16_t(&i2c_buffer[3]));

#ifdef DEBUG
    Serial.print("Read SGP30:\t");
    Serial.print(sgp30_eco2_stats->current_sample());
    Serial.print("\t");
    Serial.print(sgp30_tvoc_stats->current_sample());
    Serial.print("\tMAs:\t");
    Serial.print(sgp30_eco2_stats->current_average());
    Serial.print("\t");
    Serial.println(sgp30_tvoc_stats->current_average());
#endif
  }
}

// Set SGP30 humidity compensation
void sgp30_set_absolute_humidity(float temperature, float humidity) {
  // Don't bother if we don't have the sensor
  if (!have_sgp30) return;

  // The inputs are scaled * 100 so fix that...
  temperature /= 100;
  humidity /= 100;

  // Shamelessly copied from Adafruit examples... ahem... ;)
  // Calculate absolute humidity [mg/m^3] with approximation formula.
  //
  // Approximation formula from Sensirion SGP30 Driver Integration
  // chapter 3.15

  float absolute_humidity =
      216.7f * ((humidity / 100.0f) * 6.112f *
                exp((17.62f * temperature) / (243.12f + temperature)) /
                (273.15f + temperature));  // [g/m^3]
  absolute_humidity *= 1000.0f;            // [mg/m^3]

  // Can't set value if too large
  if (absolute_humidity > 256000.0f) {
#ifdef DEBUG
    Serial.print("sgp30_set_absolute_humidity absolute too large: ");
    Serial.println(absolute_humidity);
#endif
    status_flash(1000);
  } else {
    uint16_t ah_scaled =
        (uint16_t)(((uint64_t)absolute_humidity * 256 * 16777) >> 24);

    i2c_buffer[0] = 0x20;
    i2c_buffer[1] = 0x61;
    i2c_buffer[2] = ah_scaled >> 8;
    i2c_buffer[3] = ah_scaled & 0xFF;
    i2c_buffer[4] = crc8(i2c_buffer + 2, 2);

#ifdef DEBUG
    Serial.print("Setting sgp30 absolute humidity: ");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print(humidity);
    Serial.print("\t");
    Serial.print(absolute_humidity);
    Serial.print("\t");
    Serial.print(i2c_buffer[2]);
    Serial.print("\t");
    Serial.println(i2c_buffer[3]);
#endif

    check_i2c_write(SGP30_I2C_ADDRESS, i2c_buffer, 5, 10);
  }
}

// Calculate our IAQ
uint16_t sgp30_calc_iaq(uint16_t eco2, uint16_t tvoc) {
  // Calculate both and use the highest one.
  uint16_t iaq_eco2 = LR_CO2->calc(eco2);
  uint16_t iaq_tvoc = LR_TVOC->calc(tvoc);

#ifdef DEBUG
  Serial.print("IAQ CO2: ");
  Serial.print(iaq_eco2);
  Serial.print("\tIAQ TVOC: ");
  Serial.println(iaq_tvoc);
#endif
  return iaq_eco2 > iaq_tvoc ? iaq_eco2 : iaq_tvoc;
}

// Read raw values on demand only
unsigned long sgp30_last_raw = 0;
#define SGP30_MAX_RAW_AGE 500
uint16_t sgp30_raw_h2;
uint16_t sgp30_raw_ethanol;

uint16_t sgp30_check_raw() {
#ifdef DEBUG
  Serial.println(__func__);
#endif
  // Make sure raw values are less than 'max age' old...
  unsigned long now = millis();
  if (now - sgp30_last_raw > SGP30_MAX_RAW_AGE) {
    // ... read them if so
    sgp30_last_raw = now;
    if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_MEASURE_RAW,
                           sizeof(SGP30_MEASURE_RAW), 25, 6)) {
      sgp30_raw_h2 = decode_uint16_t(&i2c_buffer[0]);
      sgp30_raw_ethanol = decode_uint16_t(&i2c_buffer[3]);

#ifdef DEBUG_MORE
      Serial.print("Read SGP30 raw\t");
      Serial.print(sgp30_raw_h2);
      Serial.print("\t");
      Serial.print(sgp30_raw_ethanol);
      Serial.print("\n");
#endif
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// SHT31 Stuff

#define SHT31_I2C_ADDRESS 0x44
bool have_sht31 = false;

unsigned long sht31_nextRead;

StatsCalc<int16_t> *sht31_temperature_stats = new StatsCalc<int16_t>();
StatsCalc<uint16_t> *sht31_humidity_stats = new StatsCalc<uint16_t>();

void sht31_init_stats() {
  sht31_temperature_stats->init_moving_average(sht31_ma_period, sht31_period);
  sht31_humidity_stats->init_moving_average(sht31_ma_period, sht31_period);

  sht31_temperature_stats->init_delta(sht31_delta_period, sht31_period);
  sht31_humidity_stats->init_delta(sht31_delta_period, sht31_period);
}

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

    sht31_init_stats();

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

const uint8_t SHT31_MEAS_HIGHREP[] = {0x24, 0x00};
const uint8_t SHT31_READSTATUS[] = {0xF3, 0x2D};
const uint8_t SHT31_HEATEREN[] = {0x30, 0x6D};  /* Heater Enable */
const uint8_t SHT31_HEATERDIS[] = {0x30, 0x66}; /* Heater Disable */
#define SHT31_REG_MSB_HEATER_BITMASK 0x20       /* Status Register Heater Bit */

bool sht31_isHeaterEnabled() {
  if (check_i2c_response(SHT31_I2C_ADDRESS, SHT31_READSTATUS,
                         sizeof(SHT31_READSTATUS), 20, 3)) {
#ifdef DEBUG_MORE
    Serial.print("Read status: ");
    Serial.print(i2c_buffer[0], HEX);
    Serial.print("/");
    Serial.println(i2c_buffer[1], HEX);
#endif
    return (i2c_buffer[0] & SHT31_REG_MSB_HEATER_BITMASK);
  }
}

void sht31_Heater(bool heater_on) {
  check_i2c_write(SHT31_I2C_ADDRESS,
                  heater_on ? SHT31_HEATEREN : SHT31_HEATERDIS, 2, 10);
}

void sht31_read() {
  // Make sure heater is always off
  if (sht31_isHeaterEnabled()) {
#ifdef DEBUG
    Serial.println("SHT31 heater is on - turning off");
#endif
    sht31_Heater(false);
    // Make sure we don't read again for another 30s
    sht31_nextRead = millis() + 30000;
    status_flash(250);
  } else {
    /* Measurement High Repeatability with Clock Stretch Disabled */
    if (check_i2c_response(SHT31_I2C_ADDRESS, SHT31_MEAS_HIGHREP,
                           sizeof(SHT31_MEAS_HIGHREP), 20, 6)) {
      // Calculations take from Adafruit library
      int32_t stemp = (int32_t)(((uint32_t)i2c_buffer[0] << 8) | i2c_buffer[1]);
      // simplified (65536 instead of 65535) integer version of:
      // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
      stemp = ((4375 * stemp) >> 14) - 4500;

      uint32_t shum = ((uint32_t)i2c_buffer[3] << 8) | i2c_buffer[4];
      // simplified (65536 instead of 65535) integer version of:
      // humidity = (shum * 100.0f) / 65535.0f;
      shum = (625 * shum) >> 12;

      // Test negative temperatures
      // stemp -= 5000;

      // Stash in stats calculator
      sht31_temperature_stats->sample(stemp);
      sht31_humidity_stats->sample(shum);

#ifdef DEBUG
      int dtemp = sht31_temperature_stats->current_delta();
      int dhum = sht31_humidity_stats->current_delta();
      Serial.print("Read SHT31:\t");
      Serial.print(stemp);
      Serial.print("\t");
      Serial.print(shum);
      Serial.print("\tMAs:\t");
      Serial.print(sht31_temperature_stats->current_average());
      Serial.print("\t");
      Serial.print(sht31_humidity_stats->current_average());
      Serial.print("\tDs:\t");
      Serial.print(dtemp);
      Serial.print("\t");
      Serial.print(dhum);
      Serial.print("\n");
#endif
      // Set SGP30 (does nothing if no sensor present)
      sgp30_set_absolute_humidity(sht31_temperature_stats->current_average(),
                                  sht31_humidity_stats->current_average());
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
#define MODBUS_HOLDING_REGISTER_COUNT 13
#define MODBUS_INPUT_REGISTER_COUNT 14

// This is the buffer for the ModbusRTUSlave object.
// It is used to store the Modbus messages.
// A size of 256 bytes is recommended, but sizes as low as 8 bytes can be
// used.
#define MODBUS_BUFFER_SIZE 64
byte buf[MODBUS_BUFFER_SIZE];

// Initilize a SoftwareSerial port.
SoftwareSerial mySerial(modbusRxPin, modbusTxPin);

// Initilize a ModbusRTUSlave.
ModbusRTUSlave modbus(mySerial, buf, MODBUS_BUFFER_SIZE);

bool modbus_restart;

void modbus_setup() {
#ifdef DEBUG
  Serial.print(__func__);
  Serial.print("\t");
  Serial.print("id: ");
  Serial.print(modbus_id);
  Serial.print("\t");
  Serial.print("baud: ");
  Serial.print(modbus_baud_rate);
  Serial.print("\n");
#endif
  // Reset restart
  modbus_restart = false;

  // Setup the SoftwareSerial port.
  mySerial.begin(modbus_baud_rate);

  // Setup the ModbusRTUSlave
  modbus.begin(modbus_id, modbus_baud_rate);

  // Configure the input registers...
  modbus.configureInputRegisters(MODBUS_INPUT_REGISTER_COUNT,
                                 (ModbusRTUSlave::WordRead)inputRegisterRead);

  // ... and holding registers.
  modbus.configureHoldingRegisters(
      MODBUS_HOLDING_REGISTER_COUNT,
      (ModbusRTUSlave::WordRead)holdingRegisterRead,
      (ModbusRTUSlave::WordWrite)holdingRegisterWrite);
}

// Read/write holding registers
long holdingRegisterRead(word address) {
  switch (address) {
    case 0:
      return SOFTWARE_VERSION;
    case 1:
      return modbus_id;
    case 2:
      return modbus_baud_rate;
    case 3:
      return sht31_period;
    case 4:
      return sht31_ma_period;
    case 5:
      return sht31_delta_period;
    case 6:
      return sgp30_period;
    case 7:
      return sgp30_ma_period;
    case 8:
      return sgp30_serial_1;
    case 9:
      return sgp30_serial_2;
    case 10:
      return sgp30_serial_3;
    case 11:
      return sgp30_iaq_baseline_eco2;
    case 12:
      return sgp30_iaq_baseline_tvoc;
  }
  // Should never get here as all registers catered for above.
  return NAN;
}

boolean holdingRegisterWrite(word address, word value) {
  switch (address) {
    case 1:
      if (modbus_id != value) {
        modbus_id = value;
#ifdef DEBUG
        Serial.print("New Modbus address: ");
        Serial.println(modbus_id);
#endif
        modbus_restart = true;
      }
      return true;
    case 2:
      // Sanity check baud rate
      switch (value) {
        case 2400:
        case 4800:
        case 9600:
        case 19200:
        case 38400:
        case 57600:
          // Looks good - save it if necessary and restart Modbus
          if (modbus_baud_rate != value) {
            modbus_baud_rate = value;
#ifdef DEBUG
            Serial.print("New Modbus baud rate: ");
            Serial.println(modbus_baud_rate);
#endif
            modbus_restart = true;
          }
          return true;
      }
      // Sanity check must have failed
      return false;
    case 3:
      if (sht31_period != value) {
        sht31_period = value;
        sht31_init_stats();
      }
      return true;
    case 4:
      if (sht31_ma_period != value) {
        sht31_ma_period = value;
        sht31_init_stats();
      }
      return true;
    case 5:
      if (sht31_delta_period != value) {
        sht31_delta_period = value;
        sht31_init_stats();
      }
      return true;
    case 6:
      if (sgp30_period != value) {
        sgp30_period = value;
        sgp30_init_stats();
      }
      return true;
    case 7:
      if (sgp30_ma_period != value) {
        sgp30_ma_period = value;
        sgp30_init_stats();
      }
      return true;
  }
  // Can't write this
  return false;
}

// This is a function that will be passed to the ModbusRTUSlave for reading
// input registers.
long inputRegisterRead(word address) {
#ifdef DEBUG_MORE
  Serial.print("inputRegisterRead : ");
  Serial.println(address);
#endif

  switch (address) {
    case 0:
      return (uint16_t)sht31_temperature_stats->current_sample();
    case 1:
      return sht31_humidity_stats->current_sample();
    case 2:
      return (uint16_t)sht31_temperature_stats->current_average();
    case 3:
      return sht31_humidity_stats->current_average();
    case 4:
      return (uint16_t)sht31_temperature_stats->current_delta();
    case 5:
      return (uint16_t)sht31_humidity_stats->current_delta();
    case 6:
      sgp30_check_raw();
      return sgp30_raw_h2;
    case 7:
      sgp30_check_raw();
      return sgp30_raw_ethanol;
    case 8:
      return sgp30_eco2_stats->current_sample();
    case 9:
      return sgp30_tvoc_stats->current_sample();
    case 10:
      return sgp30_calc_iaq(sgp30_eco2_stats->current_sample(),
                            sgp30_tvoc_stats->current_sample());
    case 11:
      return sgp30_eco2_stats->current_average();
    case 12:
      return sgp30_tvoc_stats->current_average();
    case 13:
      return sgp30_calc_iaq(sgp30_eco2_stats->current_average(),
                            sgp30_tvoc_stats->current_average());
  }
  return NAN;
}

//////////////////////////////////////////////////////////////////////////////
// Setup

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  delay(100);
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
  // Restart Modbus interface if necessary (after parameter change).
  if (modbus_restart) {
#ifdef DEBUG
    Serial.println("Modbus restart...");
#endif
    modbus_setup();
  }

  // Poll for Modbus RTU requests from the master device which will
  // automatically run the inputRegisterRead function as needed.
  //
  // Interleave with I2C stuff.

  modbus.poll();
  sht31_loop();
  modbus.poll();
  sgp30_loop();
}
