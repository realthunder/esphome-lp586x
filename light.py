from esphome import core, pins
import esphome.codegen as cg
from esphome.components import light, i2c
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_OUTPUT_ID,
    CONF_MAX_REFRESH_RATE,
)
from esphome.core import CORE, HexInt

DEPENDENCIES = ["i2c"]


CODEOWNERS = ["@realthunder"]

lp586x_ns = cg.esphome_ns.namespace("lp586x")
LP586XLightOutput = lp586x_ns.class_(
    "LP586XLightOutput",
    cg.Component,
    i2c.I2CDevice,
    light.LightOutput,
    light.AddressableLight,
)

RGBOrder = lp586x_ns.enum("RGBOrder")

RGB_ORDERS = {
    "RGB": RGBOrder.ORDER_RGB,
    "RBG": RGBOrder.ORDER_RBG,
    "GRB": RGBOrder.ORDER_GRB,
    "GBR": RGBOrder.ORDER_GBR,
    "BGR": RGBOrder.ORDER_BGR,
    "BRG": RGBOrder.ORDER_BRG,
}

CONF_NUM_LEDS = "num_leds"
CONF_RGB_ORDER = "rgb_order"
CONF_RED_CURRENT = "red_current"
CONF_GREEN_CURRENT = "green_current"
CONF_BLUE_CURRENT = "blue_current"
CONF_SYNC_PIN = "sync_pin"
CONF_VIO_EN_PIN = "vio_en_pin"

def _validate(config):
    return config


CONFIG_SCHEMA = cv.All(
    light.ADDRESSABLE_LIGHT_SCHEMA.extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(LP586XLightOutput),
            cv.Required(CONF_NUM_LEDS): cv.int_range(1, 6),
            cv.Optional(CONF_RED_CURRENT, default="10"): cv.int_range(0, 0x7f),
            cv.Optional(CONF_GREEN_CURRENT, default="10"): cv.int_range(0, 0x7f),
            cv.Optional(CONF_BLUE_CURRENT, default="10"): cv.int_range(0, 0x7f),
            cv.Optional(CONF_RGB_ORDER, default="RGB"): cv.enum(RGB_ORDERS, upper=True),
            cv.Optional(CONF_MAX_REFRESH_RATE): cv.positive_time_period_microseconds,
            cv.Optional(CONF_SYNC_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_VIO_EN_PIN): pins.gpio_output_pin_schema,
        }
    )
    .extend(i2c.i2c_device_schema(default_address=0x40)),
    _validate,
)

FINAL_VALIDATE_SCHEMA = i2c.final_validate_device_schema(
    "lp586x"
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await light.register_light(var, config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    if CONF_RGB_ORDER in config:
        cg.add(var.set_rgb_order(config[CONF_RGB_ORDER]))
    if CONF_RED_CURRENT in config:
        cg.add(var.set_red_current(config[CONF_RED_CURRENT]))
    if CONF_GREEN_CURRENT in config:
        cg.add(var.set_green_current(config[CONF_GREEN_CURRENT]))
    if CONF_BLUE_CURRENT in config:
        cg.add(var.set_blue_current(config[CONF_BLUE_CURRENT]))
    if CONF_MAX_REFRESH_RATE in config:
        cg.add(var.set_max_refresh_rate(config[CONF_MAX_REFRESH_RATE]))
    if CONF_SYNC_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_SYNC_PIN])
        cg.add(var.set_sync_pin(pin))
    if CONF_VIO_EN_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_VIO_EN_PIN])
        cg.add(var.set_vio_en_pin(pin))

    cg.add(var.set_num_leds(config[CONF_NUM_LEDS]))

