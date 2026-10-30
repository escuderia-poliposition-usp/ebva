from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
METHODS = ['point_sample', 'scan_average']


def compose(context):
    method=context.launch_configurations['method']
    if method not in METHODS: raise ValueError('Unknown method: '+method)
    return [Node(package='ebva_frm_control',executable=method,output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('method'),OpaqueFunction(function=compose)])
