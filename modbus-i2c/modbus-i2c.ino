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

// Find sample within 1s. TODO: make this a register?
#define DELTA_SAMPLE_TOLERANCE 1000

struct DeltaSample {
  unsigned long timestamp;
  int value;
};

class StatsCalc {
 public:
  StatsCalc();
  void init_moving_average(int num_readings);
  void init_delta(unsigned long delta_period, int delta_num_samples);
  void sample(int input);
  int current_sample();
  int current_average();
  int current_delta();

 private:
  int last_sample;
  // Moving Average Stuff
  long ma_total;
  int ma_num_readings;
  bool ma_init;
  // Delta stuff
  unsigned long delta_period;
  int delta_num_samples;
  struct DeltaSample *delta_samples;
  int delta_current_slot;
  int delta_next_slot(int slot);
  int delta_prev_slot(int slot);
  int StatsCalc::delta_find_slot(unsigned long old_stamp);
  bool delta_init;
};

StatsCalc::StatsCalc() {
  // Don't do anything unless told to
  this->init_moving_average(0);
  this->init_delta(0, 0);
}

void StatsCalc::init_moving_average(int num_readings) {
  this->ma_num_readings = num_readings;
  this->ma_init = false;
}

// TODO: dynamically calculate delta_samples?
void StatsCalc::init_delta(unsigned long delta_period, int delta_num_samples) {
  this->delta_period = delta_period;
  this->delta_num_samples = delta_num_samples;
  this->delta_init = false;
}

void StatsCalc::sample(int input) {
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
    // Stash for delta calculation
    unsigned long now = millis();
    if (!this->delta_init) {
#ifdef DEBUG
      Serial.println("delta_init");
#endif
      this->delta_init = true;
      // Just fill all slots with current data
      int sample_buffer_size =
          this->delta_num_samples * sizeof(struct DeltaSample);
      this->delta_samples = (struct DeltaSample *)malloc(sample_buffer_size);
      if (this->delta_samples == NULL) {
        // Fatal error!
#ifdef DEBUG
        Serial.print("Could not malloc: ");
        Serial.println(sample_buffer_size);
#endif
        status_flash(120000);
      }

      for (int lp = 0; lp < this->delta_num_samples; lp++) {
        this->delta_samples[lp].timestamp = now;
        this->delta_samples[lp].value = input;
      }
      this->delta_current_slot = 0;
    } else {
      this->delta_current_slot =
          this->delta_next_slot(this->delta_current_slot);
      this->delta_samples[this->delta_current_slot].timestamp = now;
      this->delta_samples[this->delta_current_slot].value = input;
    }
  }

  // Always stash last sample
  this->last_sample = input;
}

// Return current moving average
int StatsCalc::current_average() {
  return this->ma_total / this->ma_num_readings;
}

// Return last value sampled
int StatsCalc::current_sample() { return this->last_sample; }

// Delta stuff

// You'd think that one could just use the oldest slot but... timing of all this
// isn't guaranteed to be 'quick' so it's possible that it could delay things
// and that we should use one of the previous slots.
//
// Yes, this is a little convoluted/complicated but really - what else is this
// CPU gonna be doing? ;)

int StatsCalc::current_delta() {
  int out = 0;

  // We need a reading which is older than last sample by period ms.
  unsigned long old_stamp =
      this->delta_samples[this->delta_current_slot].timestamp -
      this->delta_period;

#ifdef DEBUG_MORE
  Serial.print("Last reading: ");
  Serial.print(this->delta_samples[this->delta_current_slot].timestamp);
  Serial.print("\tslot: ");
  Serial.print(this->delta_current_slot);
  Serial.print("\tlooking for reading around: ");
  Serial.print(old_stamp);
  Serial.print("\tin: ");
  Serial.print(this->delta_num_samples);
  Serial.println(" slots");
#endif

  bool found_slot = false;
  int check_slot = this->delta_prev_slot(this->delta_current_slot);
  do {
    // To match, must be close enough to the interval.
    // TODO: what about when millis() wraps?
    if (abs(this->delta_samples[check_slot].timestamp - old_stamp) <=
        DELTA_SAMPLE_TOLERANCE) {
      // Found it
      out = this->last_sample - this->delta_samples[check_slot].value;
#ifdef DEBUG_MORE
      Serial.print("Found it in slot ");
      Serial.print(check_slot);
      Serial.print(" ts: ");
      Serial.print(this->delta_samples[check_slot].timestamp);
      Serial.print(" delta: ");
      Serial.println(out);
#endif
      found_slot = true;
    } else {
      check_slot = this->delta_prev_slot(check_slot);
    }
  } while (check_slot != this->delta_current_slot && !found_slot);

#ifdef DEBUG
  if (!found_slot) {
    Serial.println("Couldn't find old enough delta");
  }
#endif

  return out;
}

int StatsCalc::delta_next_slot(int slot) {
  return (++slot) % this->delta_num_samples;
}
int StatsCalc::delta_prev_slot(int slot) {
  return (slot > 0) ? (slot - 1) : (this->delta_num_samples - 1);
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

// Shared global buffer
uint8_t readbuffer[12];

bool check_i2c_response(uint8_t iAddr, uint8_t *pData, int wLen,
                        int16_t write_delay, int rLen) {
  // Assume failure
  bool ret = false;

  if (check_i2c_write(iAddr, pData, wLen, write_delay)) {
    if (I2CRead(&bbi2c, iAddr, readbuffer, rLen)) {
      ret = true;
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
// So this is a constant & cannot be changed.
#define SGP30_MEASURE_IAQ_PERIOD 1000

unsigned long sgp30_next_measure_iaq;
unsigned long sgp30_next_get_iaq_baseline;

// SGP30 sensor can return 'spikey' readings so 2-3 minutes is a good moving
// average period to smooth them. So let's use 2 minutes.

#define SGP30_MA_SAMPLES int(120000 / SGP30_MEASURE_IAQ_PERIOD)

StatsCalc *sgp30_eco2_stats = new StatsCalc();
StatsCalc *sgp30_tvoc_stats = new StatsCalc();

uint16_t sgp30_iaq_baseline_eco2;
uint16_t sgp30_iaq_baseline_tvoc;

// Period to get/save baseline - 12 hours
#define SGP30_GET_IAQ_BASELINE_PERIOD 43200000

#define SGP30_I2C_ADDRESS 0x58
bool have_sgp30 = false;

const uint8_t SGP30_READSERIAL[] = {0x36, 0x82};
#define SGP30_READSERIAL_DELAY 10
#define SGP30_READSERIAL_RESPLEN 9
const uint8_t SGP30_IAQ_INIT[] = {0x20, 0x03};
const uint8_t SGP30_MEASURE_IAQ[] = {0x20, 0x08};
const uint8_t SGP30_MEASURE_RAW[] = {0x20, 0x50};
const uint8_t SGP30_GET_IAQ_BASELINE[] = {0x20, 0x15};

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

    sgp30_eco2_stats->init_moving_average(SGP30_MA_SAMPLES);
    sgp30_tvoc_stats->init_moving_average(SGP30_MA_SAMPLES);

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

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_READSERIAL,
                         sizeof(SGP30_READSERIAL), SGP30_READSERIAL_DELAY,
                         SGP30_READSERIAL_RESPLEN)) {
    // The get serial ID command returns 3 words, and every word is followed
    // by an 8-bit CRC checksum. Together the 3 words constitute a unique
    // serial ID with a length of 48 bits.

#ifdef DEBUG
    Serial.print("Read serial: ");
    for (int lp = 0; lp < 9; lp++) {
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
      status_flash(500);
    } else {
      // Copy/store serial if changed...
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

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_GET_IAQ_BASELINE,
                         sizeof(SGP30_GET_IAQ_BASELINE), 10, 6)) {
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
  // command before each reading, but this requires *absolute* humidity and
  // the SHT31 used here cannot supply that.

  if (check_i2c_response(SGP30_I2C_ADDRESS, SGP30_MEASURE_IAQ,
                         sizeof(SGP30_MEASURE_IAQ), 12, 6)) {
    // Check CRCs...
    if (crc8(&readbuffer[0], 2) != readbuffer[2]) {
#ifdef DEBUG
      Serial.println("Bad eCO2 CRC");
#endif
      status_flash(1000);
    } else {
      sgp30_eco2_stats->sample(decode_uint16_t(&readbuffer[0]));
    }

    if (crc8(&readbuffer[3], 2) != readbuffer[5]) {
#ifdef DEBUG
      Serial.println("Bad TVOC CRC");
#endif
      status_flash(1000);
    } else {
      sgp30_tvoc_stats->sample(decode_uint16_t(&readbuffer[3]));
    }

#ifdef DEBUG
    Serial.print("Read SGP30:\t");
    Serial.print(sgp30_eco2_stats->current_sample());
    Serial.print("\t");
    Serial.print(sgp30_tvoc_stats->current_sample());
    Serial.print("\tMAs:\t");
    Serial.print(sgp30_eco2_stats->current_average());
    Serial.print("\t");
    Serial.print(sgp30_tvoc_stats->current_average());
    Serial.print("\n");
#endif
  }
}

// Shamelessly copied from Adafruit examples... ahem... ;)
// return absolute humidity [mg/m^3] with approximation formula
// @param temperature [°C]
// @param humidity [%RH]

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
  // approximation formula from Sensirion SGP30 Driver Integration
  // chapter 3.15
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
      // Check CRCs...
      if (crc8(&readbuffer[0], 2) != readbuffer[2]) {
#ifdef DEBUG
        Serial.println("Bad h2 CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_raw_h2 = decode_uint16_t(&readbuffer[0]);
      }

      if (crc8(&readbuffer[3], 2) != readbuffer[5]) {
#ifdef DEBUG
        Serial.println("Bad ethanol CRC");
#endif
        status_flash(1000);
      } else {
        sgp30_raw_ethanol = decode_uint16_t(&readbuffer[3]);
      }

#ifdef DEBUG
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
#define sht31_period 1000         // TODO: should come from register.
#define sht31_delta_period 30000  // TODO: should come from register.

// 30 second moving average for temperature/humidity. TODO: should come from
// register.
#define SHT31_MA_SAMPLES int(30000 / sht31_period)

StatsCalc *sht31_temperature_stats = new StatsCalc();
StatsCalc *sht31_humidity_stats = new StatsCalc();

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

    // Setup stats calculations
    sht31_temperature_stats->init_moving_average(SHT31_MA_SAMPLES);
    sht31_humidity_stats->init_moving_average(SHT31_MA_SAMPLES);

    sht31_temperature_stats->init_delta(
        sht31_delta_period, int(sht31_delta_period / sht31_period) + 1);
    sht31_humidity_stats->init_delta(
        sht31_delta_period, int(sht31_delta_period / sht31_period) + 1);

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

uint8_t SHT31_MEAS_HIGHREP[] = {0x24, 0x00};
uint8_t SHT31_READSTATUS[] = {0xF3, 0x2D};
uint8_t SHT31_HEATEREN[] = {0x30, 0x6D};  /* Heater Enable */
uint8_t SHT31_HEATERDIS[] = {0x30, 0x66}; /* Heater Disable */
#define SHT31_REG_MSB_HEATER_BITMASK 0x20 /* Status Register Heater Bit */

bool sht31_isHeaterEnabled() {
  if (check_i2c_response(SHT31_I2C_ADDRESS, SHT31_READSTATUS,
                         sizeof(SHT31_READSTATUS), 20, 3)) {
#ifdef DEBUG_MORE
    Serial.print("Read status: ");
    Serial.print(readbuffer[0], HEX);
    Serial.print("/");
    Serial.println(readbuffer[1], HEX);
#endif
    if (readbuffer[2] != crc8(readbuffer, 2)) {
#ifdef DEBUG
      Serial.println("Bad CRC");
#endif
      status_flash(500);
      return 0;
    } else {
      return (readbuffer[0] & SHT31_REG_MSB_HEATER_BITMASK);
    }
  }
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
    // Make sure we don't read again for another 30s
    sht31_nextRead = millis() + 30000;
    status_flash(250);
  } else {
    /* Measurement High Repeatability with Clock Stretch Disabled */
    if (check_i2c_response(SHT31_I2C_ADDRESS, SHT31_MEAS_HIGHREP,
                           sizeof(SHT31_MEAS_HIGHREP), 20, 6)) {
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

        uint32_t shum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
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
#define MODBUS_INPUT_REGISTERS 28
const word bufSize = 128;

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
  modbus.configureInputRegisters(MODBUS_INPUT_REGISTERS,
                                 (ModbusRTUSlave::WordRead)inputRegisterRead);
}

// This is a function that will be passed to the ModbusRTUSlave for reading
// input registers.
long inputRegisterRead(word address) {
#ifdef DEBUG_MORE
  Serial.print("inputRegisterRead : ");
  Serial.println(address);
#endif

  // TODO: Clean up?
  switch (address) {
    case 10:
      return (uint16_t)sht31_temperature_stats->current_sample();
    case 11:
      return sht31_humidity_stats->current_sample();
    case 12:
      return (uint16_t)sht31_temperature_stats->current_average();
    case 13:
      return sht31_humidity_stats->current_average();
    case 14:
      return (uint16_t)sht31_temperature_stats->current_delta();
    case 15:
      return (uint16_t)sht31_humidity_stats->current_delta();
    case 20:
      sgp30_check_raw();
      return sgp30_raw_h2;
    case 21:
      sgp30_check_raw();
      return sgp30_raw_ethanol;
    case 22:
      return sgp30_eco2_stats->current_sample();
    case 23:
      return sgp30_tvoc_stats->current_sample();
    case 24:
      return sgp30_calc_iaq(sgp30_eco2_stats->current_sample(),
                            sgp30_tvoc_stats->current_sample());
    case 25:
      return sgp30_eco2_stats->current_average();
    case 26:
      return sgp30_tvoc_stats->current_average();
    case 27:
      return sgp30_calc_iaq(sgp30_eco2_stats->current_average(),
                            sgp30_tvoc_stats->current_average());
  }
#ifdef DEBUG_MORE
  Serial.println("Nothing!");
#endif
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
  // Poll for Modbus RTU requests from the master device which will
  // automatically run the inputRegisterRead function as needed.
  //
  // Interleave with I2C stuff.

  modbus.poll();
  sht31_loop();
  modbus.poll();
  sgp30_loop();
}
