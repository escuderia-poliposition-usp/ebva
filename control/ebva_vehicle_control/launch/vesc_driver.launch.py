"""Standalone VESC driver for explicitly requested direct command publication."""
from ebva_vehicle_control.components import description


def generate_launch_description():
    return description("vesc_driver", "config/vehicle/manual_core.yaml")
