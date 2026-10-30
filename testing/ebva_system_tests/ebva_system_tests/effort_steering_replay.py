"""Finite rosbag2 campaigns for effort/angle consumers, without devices."""
import argparse
import json
import math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .effort_steering_software import inspect_profile


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import RcInput
    method=fixture["method"];prefix="/"+method+"_software"
    lidar=method=="lidar_frenet_pp"
    topic=prefix+("/sensors/scan" if lidar else "/state/odometry")
    types={topic:"sensor_msgs/msg/LaserScan" if lidar else "nav_msgs/msg/Odometry",
        prefix+"/rc/input":"ebva_control_msgs/msg/RcInput"}
    events=[];origin=round(fixture["origin_ros_s"]*1e9)
    for i in range(round(fixture["duration_s"]/.04)+1):
        t=round(i*.04,6);phase=next((p for p in fixture["phases"] if p["start"]<=t<p["end"]),None)
        if phase is None or phase["mode"]=="silence":continue
        ns=origin+round(t*1e9);stamp=ns-round(.8e9) if phase["mode"]=="stale" else ns
        if lidar:
            msg=LaserScan();msg.header.frame_id="wrong_frame" if phase["mode"]=="wrong_body" else "fixture_lidar_flu"
            msg.angle_min=-3*math.pi/4;msg.angle_increment=math.pi/720
            msg.angle_max=msg.angle_min+1079*msg.angle_increment;msg.range_min=.001;msg.range_max=20.;msg.scan_time=.04
            lateral=1. if 1.6<=t<2.4 else -1. if 2.6<=t<3.3 else 0.
            msg.ranges=[1.+max(0.,lateral if msg.angle_min+j*msg.angle_increment>0 else -lateral) for j in range(1080)]
        else:
            msg=Odometry();msg.header.frame_id="fixture_map"
            msg.child_frame_id="wrong_body" if phase["mode"]=="wrong_body" else "fixture_rear_axle_flu"
            msg.pose.pose.position.x=.6*t;msg.pose.pose.orientation.w=1.
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        events.append((ns,topic,msg))
    for i in range(round(fixture["duration_s"]/.03)+1):
        t=round(i*.03,6);ns=origin+round(t*1e9);msg=RcInput()
        msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,1_000_000_000)
        msg.header.frame_id="fixture_rc";msg.sequence=i+1;msg.valid=True;msg.rf_state=RcInput.RF_UNKNOWN
        msg.reason="synthetic_replay";msg.throttle_us=1500.;msg.steering_us=1700. if 3.6<=t<4. else 1500.
        events.append((ns,prefix+"/rc/input",msg))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"),storage_id="sqlite3"),rosbag2_py.ConverterOptions("cdr","cdr"))
    for name,type_name in types.items():writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=type_name,serialization_format="cdr"))
    for ns,name,msg in sorted(events,key=lambda e:(e[0],e[1])):writer.write(name,serialize_message(msg),ns)
    del writer
    return {"kind":fixture["kind"],"messages":len(events),"topics":types}


def subscriptions(record):
    from ebva_control_msgs.msg import LongitudinalEffortSteering
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    return [(LongitudinalEffortSteering,"control/autonomy/effort_steering",lambda m:record("effort_angle",m,
        {"contract":m.contract,"effort":m.longitudinal_effort,"angle":m.steering_angle_rad})),
        (DiagnosticArray,"control/path_status",lambda m:record("path_status",m,
            {"reason":m.status[0].message if m.status else ""})),
        (Odometry,"state/odometry",lambda m:record("source_pose",m,{"body":m.child_frame_id})),
        (LaserScan,"sensors/scan",lambda m:record("source_scan",m,{}))]


def verify(events, method):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind,lo,hi):
        found=[e for e in events if e["kind"]==kind and lo<=e["t_s"]<hi]
        if not found:raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return found
    commands=rows("effort_angle",1.,5.)
    effort=.2 if method=="waypoint_control" else .4
    contract="waypoint_control_carrot_v1" if method=="waypoint_control" else "lidar_frenet_pp_v1"
    frame="fixture_map" if method=="waypoint_control" else "fixture_lidar_flu"
    if not all(e["effort"]==effort and e["contract"]==contract and math.isfinite(e["angle"]) for e in commands):
        raise RuntimeError("Original effort or geometric angle contract lost")
    if not any(e["angle"]>.01 for e in commands) or not any(e["angle"]<-.01 for e in commands):
        raise RuntimeError("Fixture did not exercise both steering signs")
    if not all(e["mode"]==SelectedControl.MANUAL for e in rows("selection",3.75,3.95)):
        raise RuntimeError("RC did not take integral priority")
    if not all(e["erpm"]==0 for e in rows("motor",3.75,3.95)) or not all(abs(e["servo"]-.6)<1e-6 for e in rows("servo",3.75,3.95)):
        raise RuntimeError("Manual command pair mixed with autonomy")
    if not all(e["mode"]==SelectedControl.AUTONOMY for e in rows("selection",4.2,4.5)):
        raise RuntimeError("Valid neutral RC did not resume autonomy")
    for lo,hi in [(5.35,5.55),(5.85,6.15),(7.5,7.9)]:
        if not all(e["erpm"]==0 for e in rows("motor",lo,hi)) or not all(e["mode"]==SelectedControl.STOP for e in rows("selection",lo,hi)):
            raise RuntimeError("Invalid/stale/silent measure did not expire the pair")
    if not all(e["erpm"]>0 for e in rows("motor",6.55,6.9)):
        raise RuntimeError("Fresh input did not recover")
    sources={e["stamp_ns"] for e in events if e["kind"] in ("source_pose","source_scan")
        and e["frame"]==frame and e["t_s"]+5000.-e["stamp_ns"]/1e9<.2
        and e.get("body","fixture_rear_axle_flu")=="fixture_rear_axle_flu"}
    source={e["stamp_ns"]:e for e in events if e["kind"]=="effort_angle"}
    if any(stamp not in sources or e["frame"]!=frame for stamp,e in source.items()):
        raise RuntimeError("Original measurement header/frame lost")
    matched=0
    for e in (e for e in events if e["kind"]=="actuator"):
        command=source.get(e["stamp_ns"])
        if command is None:continue
        servo=max(.4,min(.6,.5-.2*command["angle"]))
        if e["frame"]!=frame or abs(e["erpm"]-100*command["effort"])>1e-6 or abs(e["servo"]-servo)>1e-6:
            raise RuntimeError("Explicit artificial effort/angle mapping changed")
        matched+=1
    if matched<80:raise RuntimeError("Insufficient matched conversions")
    return {"dimensionless_longitudinal_preserved":True,"geometric_angle_both_signs":True,
        "original_measurement_header":True,"atomic_pair_conversion":True,"rc_priority":True,
        "neutral_resume":True,"invalid_input_expiry":True,"stale_input_expiry":True,
        "silence_expiry":True,"fresh_recovery":True,"matched_conversions":matched}


def run(output,method):
    from .software_replay import run_campaign
    fixture=json.loads((share()/"data/effort_steering_replay.json").read_text());fixture["method"]=method
    topic="state/odometry" if method=="waypoint_control" else "sensors/scan"
    controller={"launch":method+"_software.launch.py","subscriptions":subscriptions,
        "expected_subscribers":{topic:{method,"control_replay_observer"},
            "control/autonomy/effort_steering":{"effort_steering_adapter","control_replay_observer"},
            "control/path_status":{"control_replay_observer"}},
        "clock_kinds":["selection","backend_status","effort_steering_status"]}
    # Adapter's diagnostic heartbeat is observed without renewing autonomous output.
    from diagnostic_msgs.msg import DiagnosticArray
    original=controller["subscriptions"]
    controller["subscriptions"]=lambda record:original(record)+[(DiagnosticArray,"control/effort_steering_status",
        lambda msg:record("effort_steering_status",msg,{}))]
    return run_campaign(output,inspect_profile(method),fixture,make_bag,lambda events:verify(events,method),controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--method",choices=["waypoint_control","lidar_frenet_pp"],required=True)
    parser.add_argument("--output",type=Path,required=True);args=parser.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve(),args.method);print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1

if __name__=="__main__":raise SystemExit(main())
