#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"  // always include this first
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <Wire.h>

namespace esphome {
namespace ms5837 {

// -----------------------------
// Constants and definitions
// -----------------------------
#define MS5837_ADDR 0x76

#define MS5837_RESET 0x1E
#define MS5837_ADC_READ 0x00
#define MS5837_PROM_READ 0xA0
#define MS5837_CONVERT_D1_8192 0x4A
#define MS5837_CONVERT_D2_8192 0x5A

#define MS5837_VERSION_02BA01 0x00
#define MS5837_VERSION_02BA21 0x15
#define MS5837_VERSION_30BA26 0x1A
#define MS5837_VERSION_02BA06 0x5D
#define MS5837_VERSION_UNKNOWN 0xFF

#define MS5837_ISA_SEALEVEL_PRESSURE 101325

#define MS5837_MODE_RAW 0
#define MS5837_MODE_ALTITUDE 1
#define MS5837_MODE_DEPTH 2

#define MS5837_UNITS_TEMP_C 0
#define MS5837_UNITS_TEMP_F 1
#define MS5837_UNITS_TEMP_K 2
#define MS5837_UNITS_TEMP_R 3
#define MS5837_UNITS_PRESS_HPA 0
#define MS5837_UNITS_PRESS_PA 1
#define MS5837_UNITS_PRESS_KPA 2
#define MS5837_UNITS_PRESS_INHG 3
#define MS5837_UNITS_ALT_M 0
#define MS5837_UNITS_ALT_FT 1
#define MS5837_UNITS_ALT_CM 2
#define MS5837_UNITS_ALT_IN 3

#define MS5837_OSR_8192 0
#define MS5837_OSR_4096 1
#define MS5837_OSR_2048 2
#define MS5837_OSR_1024 3
#define MS5837_OSR_512 4
#define MS5837_OSR_256 5

static const uint8_t CONVERSION_TIME[] = {18, 9, 5, 3, 2, 1};

// -----------------------------
// Main sensor class
// -----------------------------
class MS5837Sensor : public PollingComponent {
 public:
  sensor::Sensor *temperature_sensor = new sensor::Sensor();
  sensor::Sensor *pressure_sensor = new sensor::Sensor();
  sensor::Sensor *altitude_sensor = new sensor::Sensor();

  explicit MS5837Sensor(uint32_t update_ms = 60000,
                        uint8_t mode = MS5837_MODE_RAW,
                        uint8_t osr = MS5837_OSR_8192)
      : PollingComponent(update_ms),
        b_initialized_(false),
        fluid_density_(997.0f),
        atmospheric_press_(MS5837_ISA_SEALEVEL_PRESSURE),
        mode_(mode),
        osr_(osr),
        avg_runs_(1),
        external_press_(false),
        temp_units_(MS5837_UNITS_TEMP_C),
        press_units_(MS5837_UNITS_PRESS_HPA),
        alt_units_(MS5837_UNITS_ALT_M),
        temp_offset_(0.0f),
        press_offset_(0.0f) {}

  float get_setup_priority() const override {
    return setup_priority::AFTER_CONNECTION;
  }

  void setup() override;
  void update() override;
  void dump_config() override;

  // Configuration helpers
  void set_units(uint8_t temp_u, uint8_t press_u, uint8_t alt_u) {
    temp_units_ = temp_u;
    press_units_ = press_u;
    alt_units_ = alt_u;
  }

  void set_density(float density) { fluid_density_ = density; }

  void set_offsets(float temp_c, float press_hpa) {
    temp_offset_ = temp_c;
    press_offset_ = press_hpa;
  }

  void set_avg_count(uint8_t count) { avg_runs_ = count; }

 protected:
  // Internal functions
  uint8_t read_and_calc_values();
  void calculate();
  void invalidate_sensors();
  float convert_alt(float val);
  float convert_temp(float val);
  float convert_press(float val);
  float pressure();
  float temperature();
  float depth(float f_pressure_hpa);
  float altitude(float f_pressure_hpa);
  uint8_t crc4(uint16_t n_prom[]);

  // Internal state
  uint8_t model_;
  bool b_initialized_;
  uint8_t mode_;
  uint8_t avg_runs_;
  uint8_t osr_;

  uint8_t press_entity_units_;
  bool external_press_;
  bool external_press_valid_;

  uint8_t temp_units_;
  uint8_t press_units_;
  uint8_t alt_units_;

  float temp_offset_;
  float press_offset_;

  float fluid_density_;
  float atmospheric_press_;

  uint16_t C_[8];
  uint32_t D1_pres_, D2_temp_;
  int32_t TEMP_;
  int32_t P_;
};

}  // namespace ms5837
}  // namespace esphome
