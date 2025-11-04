#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "MS5837_Component.h"

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
  std::unique_ptr<MS5837_Component> driver_;
};

}  // namespace ms5837
}  // namespace esphome
