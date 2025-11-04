#include "ms5837.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"  // App.feed_wdt()

namespace esphome {
namespace ms5837 {

static const char *const TAG = "ms5837.sensor";

// Small, watchdog-friendly delay to avoid long blocking sleeps
static inline void wd_delay_ms(uint32_t total_ms) {
  const uint32_t slice = 5;  // keep it short to avoid starving other tasks
  while (total_ms > 0) {
    App.feed_wdt();
    const uint32_t d = (total_ms > slice) ? slice : total_ms;
    delay(d);
    total_ms -= d;
  }
}

void MS5837Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MS5837 (0x%02X)...", this->get_i2c_address());

  // --- Safe presence probe with timeout (no hangs even if SDA/SCL low) ---
  uint8_t dummy = 0;
  i2c::ErrorCode err = i2c::ERROR_UNKNOWN;
  uint32_t start = millis();
  while (millis() - start < 300) {  // 300 ms max wait
    err = this->write(&dummy, 0);
    if (err == i2c::ERROR_OK)
      break;
    App.feed_wdt();
    delay(5);
  }

  if (err != i2c::ERROR_OK) {
    ESP_LOGW(TAG,
             "MS5837 not responding after 300 ms (err %d), skipping init to avoid lockup.",
             err);
    this->mark_failed();
    this->status_set_warning();
    return;
  }

  // Clamp OSR index
  if (this->osr_ > 5) {
    ESP_LOGW(TAG, "Invalid OSR index %u; clamping to 0 (8192).", this->osr_);
    this->osr_ = 0;
  }

  // Reset
  uint8_t cmd = MS5837_RESET;
  err = this->write(&cmd, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Reset command failed (I2C err %d)", err);
    this->mark_failed();
    return;
  }
  wd_delay_ms(10);

  // Read calibration coefficients
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t reg = MS5837_PROM_READ + i * 2;
    uint8_t data[2] = {0};

    err = this->write(&reg, 1);
    if (err != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "PROM write failed at index %u (err %d)", i, err);
      this->status_set_warning();
      return;
    }

    err = this->read(data, 2);
    if (err != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "PROM read failed at index %u (err %d)", i, err);
      this->status_set_warning();
      return;
    }

    this->C_[i] = (data[0] << 8) | data[1];
  }

  // CRC validation
  uint8_t crc_read = C_[0] >> 12;
  uint8_t crc_calc = crc4(C_);
  if (crc_read != crc_calc) {
    ESP_LOGE(TAG,
             "CRC mismatch (read=%u calc=%u); leaving component in warning state.",
             crc_read, crc_calc);
    this->status_set_warning();
    return;
  }

  this->model_ = (C_[0] >> 5) & 0x7F;
  this->b_initialized_ = true;
  this->status_clear_warning();
  ESP_LOGCONFIG(TAG, "MS5837 ready, version 0x%02X", this->model_);
}

void MS5837Sensor::update() {
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Skipping update: device previously marked failed.");
    return;
  }

  if (!this->b_initialized_) {
    ESP_LOGW(TAG, "MS5837 not initialized, retrying setup()");
    this->setup();
    if (!this->b_initialized_) return;
  }

  if (!this->read_and_calc_values()) {
    ESP_LOGW(TAG, "Sensor read failed; publishing NaN and keeping node responsive.");
    this->invalidate_sensors();
    this->status_set_warning();
    return;
  }

  const float t = this->temperature();
  const float p = this->pressure();

  this->temperature_sensor->publish_state(this->convert_temp(t));
  this->pressure_sensor->publish_state(this->convert_press(p));

  if (this->mode_ == MS5837_MODE_ALTITUDE)
    this->altitude_sensor->publish_state(this->convert_alt(this->altitude(p)));
  else if (this->mode_ == MS5837_MODE_DEPTH)
    this->altitude_sensor->publish_state(this->convert_alt(this->depth(p)));

  this->status_clear_warning();
}

void MS5837Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MS5837 Sensor:");
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor);
  LOG_SENSOR("  ", "Altitude/Depth", this->altitude_sensor);
}

uint8_t MS5837Sensor::read_and_calc_values() {
  uint8_t cmd;
  uint8_t data[3];
  i2c::ErrorCode err;

  // --- Pressure conversion ---
  cmd = MS5837_CONVERT_D1_8192 - (this->osr_ * 2);
  err = this->write(&cmd, 1);
  if (err != i2c::ERROR_OK) return 0;
  wd_delay_ms(CONVERSION_TIME[this->osr_]);  // short-sliced delay

  cmd = MS5837_ADC_READ;
  err = this->write(&cmd, 1);
  if (err != i2c::ERROR_OK) return 0;
  err = this->read(data, 3);
  if (err != i2c::ERROR_OK) return 0;
  this->D1_pres_ = (data[0] << 16) | (data[1] << 8) | data[2];

  // --- Temperature conversion ---
  cmd = MS5837_CONVERT_D2_8192 - (this->osr_ * 2);
  err = this->write(&cmd, 1);
  if (err != i2c::ERROR_OK) return 0;
  wd_delay_ms(CONVERSION_TIME[this->osr_]);

  cmd = MS5837_ADC_READ;
  err = this->write(&cmd, 1);
  if (err != i2c::ERROR_OK) return 0;
  err = this->read(data, 3);
  if (err != i2c::ERROR_OK) return 0;
  this->D2_temp_ = (data[0] << 16) | (data[1] << 8) | data[2];

  this->calculate();
  return 1;
}

void MS5837Sensor::calculate() {
  // First-order and second-order compensation (compact form)
  int32_t dT   = D2_temp_ - ((uint32_t)C_[5] << 8);
  int64_t OFF  = ((int64_t)C_[2] << 16) + (((int64_t)C_[4] * dT) >> 7);
  int64_t SENS = ((int64_t)C_[1] << 15) + (((int64_t)C_[3] * dT) >> 8);

  TEMP_ = 2000 + ((int64_t)dT * C_[6]) / 8388608;
  P_    = ((D1_pres_ * SENS / 2097152) - OFF) / 8192;
}

float MS5837Sensor::pressure()     { return (float)P_    / 10.0f + press_offset_; }  // hPa
float MS5837Sensor::temperature()  { return (float)TEMP_ / 100.0f + temp_offset_; }  // °C

float MS5837Sensor::altitude(float f_pressure_hpa) {
  // Pressure altitude relative to sea level (ISA)
  return (1.0f - powf((f_pressure_hpa / (atmospheric_press_ / 100.0f)), 0.190284f)) * 145366.45f * 0.3048f;
}

float MS5837Sensor::depth(float f_pressure_hpa) {
  // Hydrostatic depth from ambient minus atmospheric pressure
  return ((f_pressure_hpa * 100.0f) - atmospheric_press_) / (fluid_density_ * 9.80665f);
}

void MS5837Sensor::invalidate_sensors() {
  this->temperature_sensor->publish_state(NAN);
  this->pressure_sensor->publish_state(NAN);
  this->altitude_sensor->publish_state(NAN);
}

float MS5837Sensor::convert_temp(float f) {
  if (temp_units_ == MS5837_UNITS_TEMP_F) return (f * 9.0f / 5.0f) + 32.0f;
  if (temp_units_ == MS5837_UNITS_TEMP_K) return f + 273.15f;
  if (temp_units_ == MS5837_UNITS_TEMP_R) return (f + 273.15f) * 9.0f / 5.0f;
  return f;
}

float MS5837Sensor::convert_press(float f) {
  if (press_units_ == MS5837_UNITS_PRESS_PA)   return f * 100.0f;
  if (press_units_ == MS5837_UNITS_PRESS_KPA)  return f / 10.0f;
  if (press_units_ == MS5837_UNITS_PRESS_INHG) return f * 0.02953f;
  return f; // hPa
}

float MS5837Sensor::convert_alt(float f) {
  if (alt_units_ == MS5837_UNITS_ALT_FT) return f * 3.28084f;
  if (alt_units_ == MS5837_UNITS_ALT_CM) return f * 100.0f;
  if (alt_units_ == MS5837_UNITS_ALT_IN) return f * 39.3701f;
  return f; // m
}

uint8_t MS5837Sensor::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0x00;
  n_prom[0] &= 0x0FFF;
  n_prom[7] = 0;

  for (uint8_t cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1) n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
    else              n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);

    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
      else                n_rem <<= 1;
    }
  }
  n_rem = (n_rem >> 12) & 0x000F;
  return n_rem ^ 0x00;
}

}  // namespace ms5837
}  // namespace esphome
