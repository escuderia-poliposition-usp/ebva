"""Functional interface; configuration belongs to ebva_localization."""
from ebva_localization.components import description
def generate_launch_description():
    return description('mapping_zed', 'config/slam/slam_toolbox_phase2a.yaml')
