"""Functional interface; configuration belongs to ebva_localization."""
from ebva_localization.components import description
def generate_launch_description():
    return description('ekf', 'config/slam/ekf_phase3.yaml')
