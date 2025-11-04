import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PRESSURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    UNIT_METER,
    ICON_THERMOMETER,
    ICON_GAUGE,
)

AUTO_LOAD = []
CODEOWNERS = []

# IMPORTANT: use the component namespace, not global
ms5837_ns = cg.esphome_ns.namespace("ms5837")
MS5837Sensor = ms5837_ns.class_("MS5837Sensor", cg.PollingComponent)

CONF_MODE = "mode"
CONF_OSR = "osr"
CONF_TEMPERATURE = "temperature"
CONF_PRESSURE = "pressure"
CONF_ALTITUDE = "altitude"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MS5837Sensor),
        cv.Optional("update_interval", default="60s"): cv.update_interval,
        cv.Optional(CONF_MODE, default=0): cv.int_,
        cv.Optional(CONF_OSR, default=0): cv.int_,
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
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    upd_ms = int(
        cv.ensure_list(
            cv.positive_time_period_milliseconds(config["update_interval"])
        )[0].total_milliseconds
    )
    mode = config.get(CONF_MODE, 0)
    osr = config.get(CONF_OSR, 0)

    var = cg.new_Pvariable(config[CONF_ID], upd_ms, mode, osr)
    await cg.register_component(var, config)

    if CONF_TEMPERATURE in config:
        await sensor.register_sensor(var.temperature_sensor, config[CONF_TEMPERATURE])
    if CONF_PRESSURE in config:
        await sensor.register_sensor(var.pressure_sensor, config[CONF_PRESSURE])
    if CONF_ALTITUDE in config:
        await sensor.register_sensor(var.altitude_sensor, config[CONF_ALTITUDE])
