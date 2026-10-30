"""Metadata for the software-only manual campaign; no device provider."""
from .resources import fixture_data, profile_path
NAMESPACE = "ebva"
CORE = (("ebva_vehicle_control","command_authority","command_authority"),
        ("ebva_vehicle_control","vesc_backend","vesc_backend"))

def inspect_profile():
    data = fixture_data("manual_core")
    params = {key.split("/")[-1]: value["ros__parameters"] for key,value in data.items()}
    if params["command_authority"]["rc_publisher_name"] != "rosbag2_player":
        raise ValueError("Replay must use synthetic RC")
    return ({"profile":"manual_core","namespace":NAMESPACE,"nodes":[list(row) for row in CORE],
             "profile_file":profile_path("manual_core"),"devices":[],"physical_actuation":False},
            params,None)
