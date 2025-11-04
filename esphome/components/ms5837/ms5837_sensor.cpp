#include "ms5837_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ms5837 {

static const char *const TAG = "ms5837.sensor";

void MS5837Sensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MS5837 sensor...");

  this->driver_ = std::make_unique<MS5837_Component>(
      60000, MS5837_MODE_ALTITUDE, MS5837_OSR_1024);
  this->driver_->SetUnits(MS5837_UNITS_TEMP_C,
                          MS5837_UNITS_PRESS_KPA,
                          MS5837_UNITS_ALT_M);
  this->driver_->SetOffsets(0.0f, 0.0f);
  this->driver_->SetResultsAvgCount(3);

  this->driver_->temperature_sensor = this->temperature_sensor;
  this->driver_->pressure_sensor = this->pressure_sensor;
  this->driver_->altitude_sensor = this->altitude_sensor;

  this->driver_->setup();
}

void MS5837Sensor::update() {
  if (this->driver_)
    this->driver_->update();
}

void MS5837Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "MS5837 Sensor:");
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor);
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor);
  LOG_SENSOR("  ", "Altitude", this->altitude_sensor);
}

}  // namespace ms5837
}  // namespace esphome
