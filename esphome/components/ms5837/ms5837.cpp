#include "ms5837.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ms5837 {

static const char *const TAG = "ms5837.sensor";

void MS5837Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MS5837 sensor via I2C (addr 0x%02X)...", this->address_);

  uint8_t cmd = MS5837_RESET;
  if (this->write(&cmd, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Reset command failed");
    return;
  }
  delay(10);

  // Read calibration coefficients
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t reg = MS5837_PROM_READ + i * 2;
    uint8_t data[2];
    if (this->write(&reg, 1) != i2c::ERROR_OK || this->read(data, 2) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "Failed reading calibration coefficient %u", i);
      return;
    }
    this->C_[i] = (data[0] << 8) | data[1];
  }

  uint8_t crc_read = C_[0] >> 12;
  uint8_t crc_calc = crc4(C_);
  if (crc_read != crc_calc) {
    ESP_LOGE(TAG, "CRC mismatch (read=%u calc=%u)", crc_read, crc_calc);
    return;
  }

  uint8_t version = (C_[0] >> 5) & 0x7F;
  this->model_ = version;
  this->b_initialized_ = true;

  ESP_LOGCONFIG(TAG, "MS5837 ready, version 0x%02X", version);
}

void MS5837Sensor::update() {
  if (!this->b_initialized_) {
    ESP_LOGW(TAG, "MS5837 not initialized, retrying...");
    this->setup();
    if (!this->b_initialized_) return;
  }

  if (!this->read_and_calc_values()) {
    ESP_LOGE(TAG, "Read failed");
    this->invalidate_sensors();
    return;
  }

  float t = this->temperature();
  float p = this->pressure();

  this->temperature_sensor->publish_state(this->convert_temp(t));
  this->pressure_sensor->publish_state(this->convert_press(p));

  if (this->mode_ == MS5837_MODE_ALTITUDE)
    this->altitude_sensor->publish_state(this->convert_alt(this->altitude(p)));
  else if (this->mode_ == MS5837_MODE_DEPTH)
    this->altitude_sensor->publish_state(this->convert_alt(this->depth(p)));
}

void MS5837Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MS5837 Sensor:");
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor);
  LOG_SENSOR("  ", "Altitude/Depth", this->altitude_sensor);
}

uint8_t MS5837Sensor::read_and_calc_values() {
  uint8_t cmd;

  // Pressure conversion
  cmd = MS5837_CONVERT_D1_8192 - (this->osr_ * 2);
  if (this->write(&cmd, 1) != i2c::ERROR_OK) return 0;
  delay(CONVERSION_TIME[this->osr_]);

  cmd = MS5837_ADC_READ;
  uint8_t data[3];
  if (this->write(&cmd, 1) != i2c::ERROR_OK || this->read(data, 3) != i2c::ERROR_OK) return 0;
  this->D1_pres_ = (data[0] << 16) | (data[1] << 8) | data[2];

  // Temperature conversion
  cmd = MS5837_CONVERT_D2_8192 - (this->osr_ * 2);
  if (this->write(&cmd, 1) != i2c::ERROR_OK) return 0;
  delay(CONVERSION_TIME[this->osr_]);

  cmd = MS5837_ADC_READ;
  if (this->write(&cmd, 1) != i2c::ERROR_OK || this->read(data, 3) != i2c::ERROR_OK) return 0;
  this->D2_temp_ = (data[0] << 16) | (data[1] << 8) | data[2];

  this->calculate();
  return 1;
}

void MS5837Sensor::calculate() {
  int32_t dT = D2_temp_ - ((uint32_t)C_[5] << 8);
  int64_t OFF = ((int64_t)C_[2] << 16) + (((int64_t)C_[4] * dT) >> 7);
  int64_t SENS = ((int64_t)C_[1] << 15) + (((int64_t)C_[3] * dT) >> 8);

  TEMP_ = 2000 + ((int64_t)dT * C_[6]) / 8388608;
  P_ = ((D1_pres_ * SENS / 2097152) - OFF) / 8192;
}

float MS5837Sensor::pressure() { return (float)P_ / 10.0f + press_offset_; }
float MS5837Sensor::temperature() { return (float)TEMP_ / 100.0f + temp_offset_; }

float MS5837Sensor::altitude(float f_pressure_hpa) {
  return (1.0f - powf((f_pressure_hpa / (atmospheric_press_ / 100.0f)), 0.190284f)) * 145366.45f * 0.3048f;
}

float MS5837Sensor::depth(float f_pressure_hpa) {
  return ((f_pressure_hpa * 100.0f) - atmospheric_press_) / (fluid_density_ * 9.80665f);
}

void MS5837Sensor::invalidate_sensors() {
  this->temperature_sensor->publish_state(NAN);
  this->pressure_sensor->publish_state(NAN);
  this->altitude_sensor->publish_state(NAN);
}

float MS5837Sensor::convert_temp(float fTemp) {
  if (temp_units_ == MS5837_UNITS_TEMP_F) return (fTemp * 9.0f / 5.0f) + 32.0f;
  if (temp_units_ == MS5837_UNITS_TEMP_K) return fTemp + 273.15f;
  if (temp_units_ == MS5837_UNITS_TEMP_R) return (fTemp + 273.15f) * 9.0f / 5.0f;
  return fTemp;
}

float MS5837Sensor::convert_press(float fPress) {
  if (press_units_ == MS5837_UNITS_PRESS_PA) return fPress * 100.0f;
  if (press_units_ == MS5837_UNITS_PRESS_KPA) return fPress / 10.0f;
  if (press_units_ == MS5837_UNITS_PRESS_INHG) return fPress * 0.02953f;
  return fPress;
}

float MS5837Sensor::convert_alt(float fAlt) {
  if (alt_units_ == MS5837_UNITS_ALT_FT) return fAlt * 3.28084f;
  if (alt_units_ == MS5837_UNITS_ALT_CM) return fAlt * 100.0f;
  if (alt_units_ == MS5837_UNITS_ALT_IN) return fAlt * 39.3701f;
  return fAlt;
}

uint8_t MS5837Sensor::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0x00;
  n_prom[0] &= 0x0FFF;
  n_prom[7] = 0;

  for (uint8_t cnt = 0; cnt < 16; cnt++) {
    if (cnt % 2 == 1)
      n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
    else
      n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);

    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem = (n_rem >> 12) & 0x000F;
  return n_rem ^ 0x00;
}

}  // namespace ms5837
}  // namespace esphome
