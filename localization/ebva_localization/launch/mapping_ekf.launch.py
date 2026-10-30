"""Functional interface; configuration belongs to ebva_localization."""
from ebva_localization.components import description
def generate_launch_description():
    return description('mapping_ekf', 'config/slam/slam_toolbox_phase3.yaml')
