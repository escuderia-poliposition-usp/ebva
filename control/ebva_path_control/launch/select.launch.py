from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ebva_path_control.profiles import selected_nodes, CSV_CONTRACT_PARAMS
from ebva_path_control.waypoint_pp.contract import PATH_CONTRACT_PARAMS
from ebva_path_control.effort_contract import WAYPOINT_EFFORT_PARAMS, LIDAR_EFFORT_PARAMS
from ebva_path_control.stanley_frenet.contract import STANLEY_CONTRACT_PARAMS


def compose(context):
    v = context.launch_configurations
    names = {'csv_frenet_pp': CSV_CONTRACT_PARAMS, 'waypoint_pp': PATH_CONTRACT_PARAMS,
             'waypoint_control': WAYPOINT_EFFORT_PARAMS, 'lidar_frenet_pp': LIDAR_EFFORT_PARAMS, 'stanley_frenet': STANLEY_CONTRACT_PARAMS}.get(v['method'], ())
    contract = {name: v[name] for name in names}
    return selected_nodes(v['method'], v['csv_path'], v['control_mode'], **contract)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method'),
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument('control_mode', default_value='carrot'),
        *[DeclareLaunchArgument(name, default_value='') for name in dict.fromkeys((*CSV_CONTRACT_PARAMS, *PATH_CONTRACT_PARAMS, *WAYPOINT_EFFORT_PARAMS, *LIDAR_EFFORT_PARAMS, *STANLEY_CONTRACT_PARAMS))],
        OpaqueFunction(function=compose)])
