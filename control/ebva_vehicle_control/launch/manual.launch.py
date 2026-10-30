"""Functional interface; configuration belongs to ebva_vehicle_control."""
from ebva_vehicle_control.components import description
def generate_launch_description():
    return description('manual', 'config/vehicle/manual_core.yaml')
