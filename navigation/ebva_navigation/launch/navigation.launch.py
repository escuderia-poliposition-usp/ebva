"""Functional interface; configuration belongs to ebva_navigation."""
from ebva_navigation.components import description
def generate_launch_description():
    return description('navigation', 'config/nav2_params.yaml')
