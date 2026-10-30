"""Functional interface; configuration belongs to ebva_sensors."""
from ebva_sensors.components import description
def generate_launch_description():
    return description('imu_si', 'config/imu_si.yaml')
