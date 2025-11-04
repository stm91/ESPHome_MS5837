// esphome/components/ms5837/ms5837_sensor.h
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "ms5837.h"              // your driver class & constants live here
#include <memory>

namespace esphome {
namespace ms5837 {

class MS5837Sensor : public PollingComponent {
 public:
  sensor::Sensor *temperature_sensor = new sensor::Sensor();
  sensor::Sensor *pressure_sensor = new sensor::Sensor();
  sensor::Sensor *altitude_sensor = new sensor::Sensor();

  void setup() override;
  void update() override;
  void dump_config() override;

 private:
  std::unique_ptr<MS5837_Component> driver_;   // <-- needed by your .cpp
};

}  // namespace ms5837
}  // namespace esphome
