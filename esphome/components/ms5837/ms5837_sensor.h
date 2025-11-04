#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "Arduino.h"
#include <Wire.h>

namespace esphome {
namespace ms5837 {

class MS5837Sensor : public PollingComponent {
 public:
  sensor::Sensor *temperature_sensor = new sensor::Sensor();
  sensor::Sensor *pressure_sensor = new sensor::Sensor();
  sensor::Sensor *altitude_sensor = new sensor::Sensor();

  explicit MS5837Sensor(uint32_t update_ms = 60000,
                        uint8_t mode = MS5837_MODE_RAW,
                        uint8_t osr = MS5837_OSR_8192)
      : PollingComponent(update_ms), mode_(mode), osr_(osr) {}

  void setup() override;
  void update() override;
  void dump_config() override;

 private:
  uint8_t mode_;
  uint8_t osr_;
};

}  // namespace ms5837
}  // namespace esphome
