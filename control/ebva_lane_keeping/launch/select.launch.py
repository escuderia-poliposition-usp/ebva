from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def compose(context):
    v=context.launch_configurations
    if v['v1_compat'] not in ('true','false'): raise ValueError('Expected boolean v1_compat')
    return [Node(package='ebva_lane_keeping',executable='lane_keeping',
                 parameters=[{'v1_compat':v['v1_compat']=='true','odom_topic':v['odom_topic']}],
                 remappings=[('/ackermann_cmd',v['command_topic'])],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('v1_compat',default_value='false'),
                              DeclareLaunchArgument('odom_topic',default_value='none'),
                              DeclareLaunchArgument('command_topic',default_value='/ackermann_cmd'),
                              OpaqueFunction(function=compose)])
