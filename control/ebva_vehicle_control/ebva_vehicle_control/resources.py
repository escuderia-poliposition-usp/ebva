from pathlib import Path
def share(package="ebva_vehicle_control"):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))
