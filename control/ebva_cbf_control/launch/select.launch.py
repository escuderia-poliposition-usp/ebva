"""Legacy filter-only interface; physical routing belongs to vehicle control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ebva_cbf_control.profiles import selected_nodes


def compose(context):
    values = context.launch_configurations
    if any(key in values for key in ("rc_input", "vehicle_output")):
        raise ValueError("rc_input/vehicle_output were retired; use rc_acquisition, authority and backend")
    return selected_nodes(values["model"])


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("model"), OpaqueFunction(function=compose)])
