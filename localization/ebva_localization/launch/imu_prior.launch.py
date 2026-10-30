"""Functional interface; configuration belongs to ebva_localization."""
from ebva_localization.components import description
def generate_launch_description():
    return description('imu_prior', 'config/legacy_imu_prior.yaml')
