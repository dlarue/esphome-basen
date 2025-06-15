import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, CONF_ICON

from . import CONF_BASEN_BMS_ID, BasenBMS

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_BMS_VERSION = "bms_version"
CONF_BARCODE = "barcode"
CONF_STATUS_BITMASK = "status_bitmask"
CONF_STATUS = "status"
CONF_CELL_BALANCING = "cell_balancing_bitmask"

TEXT_SENSORS = [
    CONF_BMS_VERSION,
    CONF_BARCODE,
    CONF_STATUS_BITMASK,
    CONF_STATUS,
    CONF_CELL_BALANCING
]


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_BMS_VERSION): text_sensor.text_sensor_schema(
            icon="mdi:chip"
        ),
        cv.Optional(CONF_BARCODE): text_sensor.text_sensor_schema(
            icon="mdi:barcode"
        ),
        cv.Optional(CONF_STATUS_BITMASK): text_sensor.text_sensor_schema(
             icon="mdi:information"
        ),
        cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
            icon="mdi:information-outline"
        ),
        cv.Optional(CONF_CELL_BALANCING): text_sensor.text_sensor_schema(
            icon="mdi:scale-balance"
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key in TEXT_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            yield text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
