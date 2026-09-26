import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_DEVICE_CLASS,
    CONF_ID,
    CONF_NAME,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WEIGHT,
    ICON_BATTERY,
    ICON_SCALE,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_KILOGRAM,
    UNIT_PERCENT,
)

AUTO_LOAD = ["wii_balance_board"]
DEPENDENCIES = ["wii_balance_board"]

CONF_WEIGHT = "weight"
CONF_TEMPERATURE = "temperature_sensor"
CONF_REF_TEMPERATURE = "reference_temperature_sensor"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id("WiiBalanceBoard"),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(CONF_REF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_BATTERY,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_BATTERY,
        ),
        cv.Optional(CONF_WEIGHT): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOGRAM,
            icon=ICON_SCALE,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_WEIGHT,
        ),
    }
)


async def to_code(config):
    wbb = await cg.get_variable(config[cv.GenerateID()])
    
    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(wbb.set_temperature_sensor(sens))
    
    if CONF_REF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_REF_TEMPERATURE])
        cg.add(wbb.set_reference_temperature_sensor(sens))
    
    if CONF_BATTERY_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(wbb.set_battery_level(sens))
    
    if CONF_WEIGHT in config:
        sens = await sensor.new_sensor(config[CONF_WEIGHT])
        cg.add(wbb.set_weight(sens))
