import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
# from esphome.components import spi
from esphome.const import CONF_ID, CONF_RESET_PIN, CONF_CS_PIN, CONF_MOSI_PIN, CONF_MISO_PIN, CONF_CLK_PIN
from .. import delumo_ns

CONF_FSEL_PIN = 'fsel_pin'

DelumoOutput = delumo_ns.class_("DelumoOutput", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.declare_id(DelumoOutput),
        cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_FSEL_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_MISO_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_MOSI_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_CLK_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID]) 
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_FSEL_PIN])
    cg.add(var.set_fsel_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
    cg.add(var.set_reset_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_MOSI_PIN])
    cg.add(var.set_mosi_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_MISO_PIN])
    cg.add(var.set_miso_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_CLK_PIN])
    cg.add(var.set_sclk_pin(pin))
    pin = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    cg.add(var.set_cs_pin(pin))

    # await spi.register_spi_device(var, config)
