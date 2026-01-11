import json

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, switch, uart
from esphome.components import text_sensor as text_sensor_
from esphome.const import CONF_ID, CONF_RESET_PIN
from esphome.util import Registry

from ..aqara_fp2_accel import AqaraFP2Accel

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "text_sensor", "switch"]

aqara_fp2_ns = cg.esphome_ns.namespace("aqara_fp2")
FP2Component = aqara_fp2_ns.class_("FP2Component", cg.Component, uart.UARTDevice)
FP2LocationSwitch = aqara_fp2_ns.class_("FP2LocationSwitch", switch.Switch)
FP2Zone = aqara_fp2_ns.class_("FP2Zone", cg.Component)

CONF_MOUNTING_POSITION = "mounting_position"
CONF_LEFT_RIGHT_REVERSE = "left_right_reverse"
CONF_INTERFERENCE_GRID = "interference_grid"
CONF_EXIT_GRID = "exit_grid"
CONF_EDGE_GRID = "edge_grid"
CONF_ZONES = "zones"
CONF_GRID = "grid"
CONF_SENSITIVITY = "sensitivity"

CONF_TARGET_TRACKING = "target_tracking"
CONF_LOCATION_REPORT_SWITCH = "location_report_switch"

MOUNTING_POSITIONS = {
    "wall": 0x01,
    "left_corner": 0x02,
    "right_corner": 0x03,
}

SENSITIVITY_LEVELS = {
    "low": 1,
    "medium": 2,
    "high": 3,
}


def parse_ascii_grid(value):
    lines = [li.strip() for li in value.strip().splitlines() if li.strip()]

    if len(lines) != 14:
        raise cv.Invalid(f"Grid must have exactly 14 rows, got {len(lines)}")

    for i, line in enumerate(lines):
        clean_line = line.replace(" ", "")
        if len(clean_line) != 14:
            raise cv.Invalid(
                f"Row {i + 1} must have 14 characters, got {len(clean_line)}"
            )

    grid_data = bytearray(40)
    offset_row = 0
    offset_col = 2

    for r in range(14):
        line = lines[r].replace(" ", "")
        row_val = 0
        for c in range(14):
            if line[c] in ("x", "X"):
                out_c = c + offset_col
                row_val |= 1 << (15 - out_c)
        grid_data[r * 2] = (row_val >> 8) & 0xFF
        grid_data[r * 2 + 1] = row_val & 0xFF

    return list(grid_data)


def grid_to_hex_string(grid_data):
    return "".join(f"{b:02x}" for b in grid_data)


ZONE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(FP2Zone),
        cv.Required(CONF_GRID): parse_ascii_grid,
        cv.Optional(CONF_SENSITIVITY, default="medium"): cv.enum(SENSITIVITY_LEVELS),
       
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FP2Component),
            cv.Required("accel"): cv.use_id(AqaraFP2Accel),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MOUNTING_POSITION, default="left_corner"): cv.enum(
                MOUNTING_POSITIONS
            ),
            cv.Optional(CONF_LEFT_RIGHT_REVERSE, default=False): cv.boolean,
            cv.Optional(CONF_INTERFERENCE_GRID): parse_ascii_grid,
            cv.Optional(CONF_EXIT_GRID): parse_ascii_grid,
            cv.Optional(CONF_EDGE_GRID): parse_ascii_grid,
            cv.Optional(CONF_ZONES): cv.All(cv.ensure_list(ZONE_SCHEMA)),
            cv.Optional(CONF_TARGET_TRACKING): text_sensor_.text_sensor_schema(),
            cv.Optional(CONF_LOCATION_REPORT_SWITCH): switch.switch_schema(
                FP2LocationSwitch
            ),
            cv.Optional("edge_label_grid_sensor"): text_sensor_.text_sensor_schema(),
            cv.Optional("entry_exit_grid_sensor"): text_sensor_.text_sensor_schema(),
            cv.Optional("interference_grid_sensor"): text_sensor_.text_sensor_schema(),
            cv.Optional("mounting_position_sensor"): text_sensor_.text_sensor_schema(),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    zones = []
    if CONF_ZONES in config:
        for i, zone_conf in enumerate(config[CONF_ZONES]):
            var = cg.new_Pvariable(
                zone_conf[CONF_ID],
                i + 1,
                zone_conf[CONF_GRID],
                zone_conf[CONF_SENSITIVITY],
            )
            await cg.register_component(var, zone_conf)

            zones.append(var)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))

    cg.add(var.set_mounting_position(config[CONF_MOUNTING_POSITION]))
    cg.add(var.set_left_right_reverse(config[CONF_LEFT_RIGHT_REVERSE]))

    if CONF_INTERFERENCE_GRID in config:
        cg.add(var.set_interference_grid(config[CONF_INTERFERENCE_GRID]))

    if CONF_EXIT_GRID in config:
        cg.add(var.set_exit_grid(config[CONF_EXIT_GRID]))

    if CONF_EDGE_GRID in config:
        cg.add(var.set_edge_grid(config[CONF_EDGE_GRID]))

    cg.add(var.set_zones(zones))

    if CONF_TARGET_TRACKING in config:
        sens = await text_sensor_.new_text_sensor(config[CONF_TARGET_TRACKING])
        cg.add(var.set_target_tracking_sensor(sens))

    if CONF_LOCATION_REPORT_SWITCH in config:
        sw = await switch.new_switch(config[CONF_LOCATION_REPORT_SWITCH])
        cg.add(var.set_location_report_switch(sw))

    if "edge_label_grid_sensor" in config:
        sens = await text_sensor_.new_text_sensor(config["edge_label_grid_sensor"])
        cg.add(var.set_edge_label_grid_sensor(sens))
    
    if "entry_exit_grid_sensor" in config:
        sens = await text_sensor_.new_text_sensor(config["entry_exit_grid_sensor"])
        cg.add(var.set_entry_exit_grid_sensor(sens))
    
    if "interference_grid_sensor" in config:
        sens = await text_sensor_.new_text_sensor(config["interference_grid_sensor"])
        cg.add(var.set_interference_grid_sensor(sens))
    
    if "mounting_position_sensor" in config:
        sens = await text_sensor_.new_text_sensor(config["mounting_position_sensor"])
        cg.add(var.set_mounting_position_sensor(sens))

    map_config_data = {
        "mounting_position": config[CONF_MOUNTING_POSITION],
        "left_right_reverse": config[CONF_LEFT_RIGHT_REVERSE],
    }

    if CONF_INTERFERENCE_GRID in config:
        map_config_data["interference_grid"] = grid_to_hex_string(
            config[CONF_INTERFERENCE_GRID]
        )
    if CONF_EXIT_GRID in config:
        map_config_data["exit_grid"] = grid_to_hex_string(config[CONF_EXIT_GRID])
    if CONF_EDGE_GRID in config:
        map_config_data["edge_grid"] = grid_to_hex_string(config[CONF_EDGE_GRID])

    if CONF_ZONES in config:
        map_config_data["zones"] = [
            {
                "sensitivity": z[CONF_SENSITIVITY],
                "grid": grid_to_hex_string(z[CONF_GRID]),
            }
            for z in config[CONF_ZONES]
        ]

    cg.add(var.set_map_config_json(json.dumps(map_config_data, separators=(",", ":"))))

    accel = await cg.get_variable(config["accel"])
    cg.add(var.set_fp2_accel(accel))
