import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ICON, CONF_ID, ICON_SIGNAL

from . import CONF_BASEN_BMS_ID, BasenBMS

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_CONNECTED = "connected"

BINARY_SENSORS = [
    CONF_CONNECTED,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_CONNECTED): binary_sensor.BINARY_SENSOR_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(binary_sensor.BinarySensor),
                cv.Optional(CONF_ICON, default=ICON_SIGNAL): cv.icon,
            }
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            yield binary_sensor.register_binary_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
