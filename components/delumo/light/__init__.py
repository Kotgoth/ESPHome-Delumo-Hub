import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light, output
from esphome.const import CONF_OUTPUT_ID, CONF_OUTPUT
from .. import delumo_ns

CONF_SERIAL_KEY = 'serial'

BinaryLightOutput = delumo_ns.class_("DelumoLightOutput", light.LightOutput)

CONFIG_SCHEMA = light.BINARY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(BinaryLightOutput),
        # cv.Required(CONF_OUTPUT): cv.use_id(output.BinaryOutput),
        cv.Required(CONF_OUTPUT): cv.use_id(delumo_ns.class_("DelumoOutput")),
        cv.Required(CONF_SERIAL_KEY): cv.uint16_t,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)

    out = await cg.get_variable(config[CONF_OUTPUT])
    cg.add(var.set_output(out))

    cg.add(var.set_serial(config[CONF_SERIAL_KEY]))