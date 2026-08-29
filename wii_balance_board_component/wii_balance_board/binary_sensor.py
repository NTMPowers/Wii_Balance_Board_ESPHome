import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    ICON_BLUETOOTH,
)

AUTO_LOAD = ["wii_balance_board"]
DEPENDENCIES = ["wii_balance_board"]

CONF_SYNCING = "syncing"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id("WiiBalanceBoard"),
        cv.Optional(CONF_SYNCING): binary_sensor.binary_sensor_schema(
            icon=ICON_BLUETOOTH,
        ),
    }
)


async def to_code(config):
    wbb = await cg.get_variable(config[cv.GenerateID()])
    
    if CONF_SYNCING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SYNCING])
        cg.add(wbb.set_syncing(sens))
