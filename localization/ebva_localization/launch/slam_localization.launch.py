"""Functional interface; configuration belongs to ebva_localization."""
from ebva_localization.components import description
def generate_launch_description():
    return description('slam_localization', 'config/slam/slam_toolbox_localization.yaml')
