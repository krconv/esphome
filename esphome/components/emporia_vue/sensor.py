from esphome.components import sensor, i2c
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_CALIBRATION,
    DEVICE_CLASS_ENERGY,
    STATE_CLASS_MEASUREMENT,
    UNIT_WATT,
)

CODEOWNERS = ["@flaviut", "@Maelstrom96", "@krconv"]
ESP_PLATFORMS = ["esp-idf"]
DEPENDENCIES = ["i2c"]
AUTOLOAD = ["sensor"]

emporia_vue_ns = cg.esphome_ns.namespace("emporia_vue")
EmporiaVueComponent = emporia_vue_ns.class_(
    "EmporiaVueComponent", cg.PollingComponent, i2c.I2CDevice
)

PhaseConfig = emporia_vue_ns.class_("PhaseConfig")

PhaseInputWire = emporia_vue_ns.enum("PhaseInputWire")
PHASE_INPUT = {
    "BLACK": PhaseInputWire.BLACK,
    "RED": PhaseInputWire.RED,
    "BLUE": PhaseInputWire.BLUE,
}

PowerSensor = emporia_vue_ns.class_("PowerSensor", sensor.Sensor)

CTInputPort = emporia_vue_ns.enum("CTInputPort")
CT_INPUT = {
    "A": CTInputPort.A,
    "B": CTInputPort.B,
    "C": CTInputPort.C,
    "1": CTInputPort.ONE,
    "2": CTInputPort.TWO,
    "3": CTInputPort.THREE,
    "4": CTInputPort.FOUR,
    "5": CTInputPort.FIVE,
    "6": CTInputPort.SIX,
    "7": CTInputPort.SEVEN,
    "8": CTInputPort.EIGHT,
    "9": CTInputPort.NINE,
    "10": CTInputPort.TEN,
    "11": CTInputPort.ELEVEN,
    "12": CTInputPort.TWELVE,
    "13": CTInputPort.THIRTEEN,
    "14": CTInputPort.FOURTEEN,
    "15": CTInputPort.FIFTEEN,
    "16": CTInputPort.SIXTEEN,
}

CONF_PHASES = "phases"
CONF_PHASE_ID = "phase_id"
CONF_POWER = "power"
CONF_SENSOR_POLL_INTERVAL = "sensor_poll_interval"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EmporiaVueComponent),
            cv.Optional(
                CONF_SENSOR_POLL_INTERVAL, default="240ms"
            ): cv.positive_time_period_milliseconds,
            cv.Required(CONF_PHASES): cv.ensure_list(
                {
                    cv.Required(CONF_ID): cv.declare_id(PhaseConfig),
                    cv.Required(CONF_INPUT): cv.enum(PHASE_INPUT),
                    cv.Optional(CONF_CALIBRATION, default=0.022): cv.float_range(
                        0.0, 1.0
                    ),
                }
            ),
            cv.Required(CONF_POWER): cv.ensure_list(
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_WATT,
                    device_class=DEVICE_CLASS_ENERGY,
                    state_class=STATE_CLASS_MEASUREMENT,
                    accuracy_decimals=0,
                ).extend(
                    {
                        cv.GenerateID(): cv.declare_id(PowerSensor),
                        cv.Required(CONF_PHASE_ID): cv.use_id(PhaseConfig),
                        cv.Required(CONF_INPUT): cv.enum(CT_INPUT),
                    }
                )
            ),
        },
        cv.only_with_esp_idf,
    )
    .extend(cv.polling_component_schema("5s"))
    .extend(i2c.i2c_device_schema(0x64))
)


async def to_code(config):
    component_var = cg.new_Pvariable(config[CONF_ID])
    cg.add(component_var.set_sensor_poll_interval(config[CONF_SENSOR_POLL_INTERVAL]))
    await cg.register_component(component_var, config)
    await i2c.register_i2c_device(component_var, config)

    phases = []
    for phase_config in config[CONF_PHASES]:
        phase_var = cg.new_Pvariable(phase_config[CONF_ID], PhaseConfig())
        cg.add(phase_var.set_input_wire(phase_config[CONF_INPUT]))
        cg.add(phase_var.set_calibration(phase_config[CONF_CALIBRATION]))

        phases.append(phase_var)
    cg.add(component_var.set_phases(phases))

    power_sensors = []
    for power_config in config[CONF_POWER]:
        power_var = cg.new_Pvariable(power_config[CONF_ID], PowerSensor())
        phase_var = await cg.get_variable(power_config[CONF_PHASE_ID])
        cg.add(power_var.set_phase(phase_var))
        cg.add(power_var.set_input_port(power_config[CONF_INPUT]))

        await sensor.register_sensor(power_var, power_config)

        power_sensors.append(power_var)
    cg.add(component_var.set_power_sensors(power_sensors))
