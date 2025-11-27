import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import HexInt
from esphome import automation

DEPENDENCIES = ["esp32_ble"]

CONF_MESH_KEY = "mesh_key"
CONF_ADV_INTERVAL_MIN = "adv_interval_min"
CONF_ADV_INTERVAL_MAX = "adv_interval_max"
CONF_ADV_DURATION = "adv_duration"
CONF_ADV_GAP = "adv_gap"
CONF_MAX_QUEUE_SIZE = "max_queue_size"

DEFAULT_ADV_INTERVAL_MIN = 0x20
DEFAULT_ADV_INTERVAL_MAX = 0x40
DEFAULT_ADV_DURATION = 50
DEFAULT_ADV_GAP = 10
DEFAULT_MAX_QUEUE_SIZE = 100


def validate_hex_bytes(value):
    if isinstance(value, str):
        value = value.replace(" ", "")
        if len(value) != 8:
            raise cv.Invalid("Mesh key must be exactly 4 bytes (8 hex characters)")

        try:
            return HexInt(int(value, 16))
        except ValueError as err:
            raise cv.Invalid(f"Invalid hex value: {err}")
    raise cv.Invalid("Mesh key must be a string")


fastcon_ns = cg.esphome_ns.namespace("fastcon")
FastconController = fastcon_ns.class_("FastconController", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ID, default="fastcon_controller"): cv.declare_id(
            FastconController
        ),
        cv.Required(CONF_MESH_KEY): validate_hex_bytes,
        cv.Optional(
            CONF_ADV_INTERVAL_MIN, default=DEFAULT_ADV_INTERVAL_MIN
        ): cv.uint16_t,
        cv.Optional(
            CONF_ADV_INTERVAL_MAX, default=DEFAULT_ADV_INTERVAL_MAX
        ): cv.uint16_t,
        cv.Optional(CONF_ADV_DURATION, default=DEFAULT_ADV_DURATION): cv.uint16_t,
        cv.Optional(CONF_ADV_GAP, default=DEFAULT_ADV_GAP): cv.uint16_t,
        cv.Optional(
            CONF_MAX_QUEUE_SIZE, default=DEFAULT_MAX_QUEUE_SIZE
        ): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_MESH_KEY in config:
        mesh_key = config[CONF_MESH_KEY]
        key_bytes = [(mesh_key >> (i * 8)) & 0xFF for i in range(3, -1, -1)]
        cg.add(var.set_mesh_key(key_bytes))

    if config[CONF_ADV_INTERVAL_MAX] < config[CONF_ADV_INTERVAL_MIN]:
        raise cv.Invalid(
            f"adv_interval_max ({config[CONF_ADV_INTERVAL_MAX]}) must be >= "
            f"adv_interval_min ({config[CONF_ADV_INTERVAL_MIN]})"
        )

    cg.add(var.set_adv_interval_min(config[CONF_ADV_INTERVAL_MIN]))
    cg.add(var.set_adv_interval_max(config[CONF_ADV_INTERVAL_MAX]))
    cg.add(var.set_adv_duration(config[CONF_ADV_DURATION]))
    cg.add(var.set_adv_gap(config[CONF_ADV_GAP]))
    cg.add(var.set_max_queue_size(config[CONF_MAX_QUEUE_SIZE]))


# Actions for pairing
PairDeviceAction = fastcon_ns.class_("PairDeviceAction", automation.Action)
FactoryResetAction = fastcon_ns.class_("FactoryResetAction", automation.Action)


@automation.register_action(
    "fastcon.pair_device",
    PairDeviceAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(FastconController),
        cv.Required("light_id"): cv.templatable(cv.positive_int),
        cv.Optional("group_id", default=1): cv.templatable(cv.positive_int),
    })
)
async def fastcon_pair_device_to_code(config, action_id, template_arg, args):
    var = await cg.get_variable(config[CONF_ID])
    action = cg.new_Pvariable(action_id, template_arg, var)
    
    template_ = await cg.templatable(config["light_id"], args, cg.uint32)
    cg.add(action.set_light_id(template_))
    
    template_ = await cg.templatable(config["group_id"], args, cg.uint32)
    cg.add(action.set_group_id(template_))
    
    return action


@automation.register_action(
    "fastcon.factory_reset",
    FactoryResetAction,
    cv.Schema({
        cv.GenerateID(): cv.use_id(FastconController),
        cv.Required("light_id"): cv.templatable(cv.positive_int),
    })
)
async def fastcon_factory_reset_to_code(config, action_id, template_arg, args):
    var = await cg.get_variable(config[CONF_ID])
    action = cg.new_Pvariable(action_id, template_arg, var)
    
    template_ = await cg.templatable(config["light_id"], args, cg.uint32)
    cg.add(action.set_light_id(template_))
    
    return action
