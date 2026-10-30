"""Explicit composition of one path method; inspection starts no process."""
METHODS = ('waypoint_pp', 'csv_frenet_pp', 'lidar_frenet_pp', 'waypoint_control', 'waypoint_preview', 'stanley_frenet')
CSV_CONTRACT_PARAMS = ('reference_frame_id', 'body_frame_id', 'model_point',
                       'reference_contract', 'progress_origin', 'progress_direction')


def selected_nodes(method, csv_path='', control_mode='carrot', **csv_contract):
    from launch_ros.actions import Node
    if method not in METHODS:
        raise ValueError('Unknown path method: '+method)
    if control_mode not in ('carrot', 'stanley', 'pure_pursuit'):
        raise ValueError('Unknown waypoint control mode')
    params = {}
    if method == 'waypoint_pp':
        from .waypoint_pp.contract import WaypointContract, PATH_CONTRACT_PARAMS, load_waypoint_reference
        if not csv_path or set(csv_contract) != set(PATH_CONTRACT_PARAMS) or not all(csv_contract.values()):
            raise ValueError('Explicit waypoint path/frame/body contracts required')
        points = load_waypoint_reference(csv_path)
        WaypointContract(points, **csv_contract)
        params.update(csv_contract)
        params['csv_path'] = csv_path
    if method == 'csv_frenet_pp':
        from .references import load_frenet_reference
        from .csv_frenet.contract import CsvFrenetContract
        if not csv_path:
            raise ValueError('csv_path is required for csv_frenet_pp')
        if (set(csv_contract) != set(CSV_CONTRACT_PARAMS)
                or not all(csv_contract.values())):
            raise ValueError('Explicit CSV reference/frame/model/progress parameters required')
        s_ref, ey_ref = load_frenet_reference(csv_path)
        CsvFrenetContract(s_ref, ey_ref, **csv_contract)
        params.update(csv_contract)
        params['csv_path'] = csv_path
    if method == 'waypoint_control':
        from .effort_contract import WAYPOINT_EFFORT_PARAMS, WaypointEffortContract, load_effort_waypoints
        if set(csv_contract) != set(WAYPOINT_EFFORT_PARAMS) or not all(csv_contract.values()):
            raise ValueError('Explicit waypoint effort reference/body/model contracts required')
        if not csv_path:
            from ament_index_python.packages import get_package_share_directory
            from pathlib import Path
            csv_path = str(Path(get_package_share_directory('ebva_path_control'))/'data/waypoint_debug/centerline.csv')
        WaypointEffortContract(load_effort_waypoints(csv_path), **csv_contract, control_mode=control_mode)
        params.update(csv_contract)
        params.update(csv_path=csv_path, control_mode=control_mode)
    if method == 'lidar_frenet_pp':
        from .effort_contract import LIDAR_EFFORT_PARAMS, LidarEffortContract
        if set(csv_contract) != set(LIDAR_EFFORT_PARAMS) or not all(csv_contract.values()):
            raise ValueError('Explicit LiDAR scan frame/contract required')
        LidarEffortContract(**csv_contract)
        params.update(csv_contract)
    if method == 'stanley_frenet':
        from .stanley_frenet.contract import StanleyFrenetContract, STANLEY_CONTRACT_PARAMS
        if set(csv_contract) != set(STANLEY_CONTRACT_PARAMS) or not all(csv_contract.values()):
            raise ValueError('Explicit Stanley clearance/frame/body/model/reference required')
        StanleyFrenetContract(**csv_contract)
        params.update(csv_contract)
    return [Node(package='ebva_path_control', executable=method, parameters=[params], output='screen')]
