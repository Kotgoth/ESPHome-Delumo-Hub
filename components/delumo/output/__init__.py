import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi
from esphome.const import CONF_ID, CONF_MODE, CONF_RESET_PIN, CONF_CS_PIN
from .. import delumo_ns

DEPENDENCIES = ["spi"]

CONF_FSEL_PIN = 'fsel_pin'

DelumoOutput = delumo_ns.class_("DelumoOutput", cg.Component, spi.SPIDevice)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.declare_id(DelumoOutput),
        cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_FSEL_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema(cs_pin_required=True, default_data_rate='1MHz', default_mode='MODE0'))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID]) 
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_FSEL_PIN])
    cg.add(var.set_fsel_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(pin))

    await spi.register_spi_device(var, config)
