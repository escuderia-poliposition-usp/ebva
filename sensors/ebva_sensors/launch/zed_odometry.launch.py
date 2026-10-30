"""Functional interface; configuration belongs to ebva_sensors."""
from ebva_sensors.components import description
def generate_launch_description():
    return description('zed_odometry', 'config/zed_odometry.yaml')
