# esphome/components/ms5837/sensor.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PRESSURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    UNIT_METER,
    ICON_THERMOMETER,
    ICON_GAUGE,
)

AUTO_LOAD = ["sensor"]
DEPENDENCIES = ["i2c"]
CODEOWNERS = []

# IMPORTANT: namespace must match the component folder name
ms5837_ns = cg.esphome_ns.namespace("ms5837")
# Declare both base classes so register_i2c_device can locate set_i2c_bus/set_i2c_address
MS5837Sensor = ms5837_ns.class_("MS5837Sensor", cg.PollingComponent, i2c.I2CDevice)

CONF_MODE = "mode"
CONF_OSR = "osr"
CONF_AVG_COUNT = "avg_count"
CONF_TEMP_OFFSET = "temp_offset"
CONF_PRESS_OFFSET = "press_offset"
CONF_FLUID_DENSITY = "fluid_density"
CONF_ATMOSPHERIC_PRESSURE = "atmospheric_pressure"
CONF_TEMPERATURE = "temperature"
CONF_PRESSURE = "pressure"
CONF_ALTITUDE = "altitude"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MS5837Sensor),
            cv.Optional(CONF_MODE, default=0): cv.int_range(min=0, max=2),
            cv.Optional(CONF_OSR, default=0): cv.int_range(min=0, max=5),
            cv.Optional(CONF_AVG_COUNT, default=1): cv.int_range(min=1, max=255),
            # Calibration trims — applied to the raw reading before unit conversion / depth calc
            cv.Optional(CONF_TEMP_OFFSET, default=0.0): cv.float_,
            cv.Optional(CONF_PRESS_OFFSET, default=0.0): cv.float_,
            # Fluid density in kg/m³ for depth mode (997 = freshwater, 1025 = seawater)
            cv.Optional(CONF_FLUID_DENSITY, default=997.0): cv.positive_float,
            # Reference atmospheric pressure in Pa for depth=0 baseline (default 101325 = ISA sea level)
            # Set this to your local atmospheric pressure for an accurate zero point without press_offset
            cv.Optional(CONF_ATMOSPHERIC_PRESSURE, default=101325.0): cv.positive_float,
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_HECTOPASCAL,
                icon=ICON_GAUGE,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_PRESSURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ALTITUDE): sensor.sensor_schema(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=2,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x76))
)

async def to_code(config):
    upd_ms = int(config[CONF_UPDATE_INTERVAL].total_milliseconds)
    var = cg.new_Pvariable(config[CONF_ID], upd_ms, config[CONF_MODE], config[CONF_OSR])
    await cg.register_component(var, config)
    # Wire the I2C bus and address (critical — without this, write/read crash)
    await i2c.register_i2c_device(var, config)

    if config[CONF_AVG_COUNT] != 1:
        cg.add(var.set_avg_count(config[CONF_AVG_COUNT]))
    if config[CONF_TEMP_OFFSET] != 0.0 or config[CONF_PRESS_OFFSET] != 0.0:
        cg.add(var.set_offsets(config[CONF_TEMP_OFFSET], config[CONF_PRESS_OFFSET]))
    if config[CONF_FLUID_DENSITY] != 997.0:
        cg.add(var.set_density(config[CONF_FLUID_DENSITY]))
    if config[CONF_ATMOSPHERIC_PRESSURE] != 101325.0:
        cg.add(var.set_atmospheric_pressure(config[CONF_ATMOSPHERIC_PRESSURE]))

    if CONF_TEMPERATURE in config:
        await sensor.register_sensor(var.temperature_sensor, config[CONF_TEMPERATURE])
    if CONF_PRESSURE in config:
        await sensor.register_sensor(var.pressure_sensor, config[CONF_PRESSURE])
    if CONF_ALTITUDE in config:
        await sensor.register_sensor(var.altitude_sensor, config[CONF_ALTITUDE])
