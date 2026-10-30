"""Functional interface; configuration belongs to ebva_sensors."""
from ebva_sensors.components import description
def generate_launch_description():
    return description('lidar_mapping_zed', 'config/lidar_mapping_zed.yaml')
