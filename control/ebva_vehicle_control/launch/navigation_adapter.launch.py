"""Functional interface; configuration belongs to ebva_vehicle_control."""
from ebva_vehicle_control.components import description
def generate_launch_description():
    return description('navigation_adapter', 'config/navigation_adapter.yaml')
