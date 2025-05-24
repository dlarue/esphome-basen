import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_BATTERY_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    #DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    #DEVICE_CLASS_DURATION,
    ICON_CURRENT_AC,
    ICON_EMPTY,
    ICON_FLASH,
    ICON_GAUGE,
    ICON_PERCENT,
    ICON_POWER,
    ICON_THERMOMETER,
    ICON_BATTERY,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

from . import CONF_BASEN_BMS_ID, BasenBMS

DEPENDENCIES = ["basen_bms"]

CODEOWNERS = ["GHswitt"]

CONF_VOLTAGE = "voltage"
CONF_CURRENT = "current"
CONF_POWER = "power"
CONF_TEMPERATURE1 = "temperature1"
CONF_TEMPERATURE2 = "temperature2"
CONF_TEMPERATURE3 = "temperature3"
CONF_TEMPERATURE4 = "temperature4"
CONF_TEMPERATURE_MOS = "temperature_mos"
CONF_TEMPERATURE_AMBIENT = "temperature_ambient"
CONF_CAPACITY = "capacity"
CONF_STATE_OF_CHARGE = "state_of_charge"
CONF_STATE_OF_HEALTH = "state_of_health"
CONF_CYCLES = "cycles"
CONF_AVG_CELL_VOLTAGE = "avg_cell_voltage"
CONF_MIN_CELL_VOLTAGE = "min_cell_voltage"
CONF_MAX_CELL_VOLTAGE = "max_cell_voltage"
CONF_DELTA_CELL_VOLTAGE = "delta_cell_voltage"
CONF_MIN_CELL_INDEX = "min_cell_index"
CONF_MAX_CELL_INDEX = "max_cell_index"
UNIT_AMPERE_HOUR = "Ah"

SENSORS = [
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_CAPACITY,
    CONF_STATE_OF_CHARGE,
    CONF_STATE_OF_HEALTH,
    CONF_CYCLES,
    CONF_TEMPERATURE1,
    CONF_TEMPERATURE2,
    CONF_TEMPERATURE3,
    CONF_TEMPERATURE4,
    CONF_TEMPERATURE_MOS,
    CONF_TEMPERATURE_AMBIENT,
    CONF_AVG_CELL_VOLTAGE,
    CONF_MIN_CELL_VOLTAGE,
    CONF_MAX_CELL_VOLTAGE,
    CONF_MIN_CELL_INDEX,
    CONF_MAX_CELL_INDEX,
    CONF_DELTA_CELL_VOLTAGE
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BASEN_BMS_ID): cv.use_id(BasenBMS),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon=ICON_FLASH,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            icon=ICON_POWER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            icon=ICON_CURRENT_AC,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_CAPACITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE_HOUR,
            icon=ICON_GAUGE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_STATE_OF_CHARGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_BATTERY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_STATE_OF_HEALTH): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_CYCLES): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
        ),
        cv.Optional(CONF_TEMPERATURE1): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE2): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE3): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE4): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE_MOS): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_TEMPERATURE_AMBIENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_AVG_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon=ICON_FLASH,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MIN_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon=ICON_FLASH,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MAX_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon=ICON_FLASH,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_DELTA_CELL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon=ICON_FLASH,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MIN_CELL_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
        cv.Optional(CONF_MAX_CELL_INDEX): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_COUNTER,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT
        ),
    }
)


def to_code(config):
    hub = yield cg.get_variable(config[CONF_BASEN_BMS_ID])
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = yield sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
