"""rosbag2 Odometry → exact pose → waypoint PP → Ackermann/RC software campaign."""
import argparse
import json
import math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .waypoint_pp_software import inspect_profile,NAMESPACE


def make_bag(output,fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from ebva_control_msgs.msg import RcInput
    prefix="/"+NAMESPACE
    types={prefix+"/state/odometry":"nav_msgs/msg/Odometry",prefix+"/rc/input":"ebva_control_msgs/msg/RcInput"}
    events=[];origin=round(fixture["origin_ros_s"]*1e9)
    for i in range(round(fixture["duration_s"]/fixture["odom_period_s"])+1):
        t=round(i*fixture["odom_period_s"],6)
        phase=next((p for p in fixture["phases"] if p["start"]<=t<p["end"]),None)
        if phase is None or phase["mode"]=="silence":continue
        ns=origin+round(t*1e9);stamp=ns-round(.8e9) if phase["mode"]=="stale" else ns
        msg=Odometry();msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        msg.header.frame_id="fixture_map";msg.child_frame_id="wrong_body" if phase["mode"]=="wrong_body" else "fixture_rear_axle_flu"
        angle=.12*t;yaw=angle+math.pi/2+phase["yaw_offset"]
        msg.pose.pose.position.x=2*math.cos(angle);msg.pose.pose.position.y=2*math.sin(angle)
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        # Synthetic source covariance: adapter must copy, never estimate it.
        msg.pose.covariance=[.01 if j%7==0 else 0. for j in range(36)]
        msg.twist.twist.linear.x=.24
        events.append((ns,prefix+"/state/odometry",msg))
    for i in range(round(fixture["duration_s"]/fixture["rc_period_s"])+1):
        t=round(i*fixture["rc_period_s"],6);ns=origin+round(t*1e9)
        rc=RcInput();rc.header.stamp.sec,rc.header.stamp.nanosec=divmod(ns,1_000_000_000)
        rc.header.frame_id="fixture_rc";rc.sequence=i+1;rc.valid=True;rc.rf_state=RcInput.RF_UNKNOWN
        rc.reason="synthetic_replay";rc.throttle_us=1500.
        lo,hi=fixture["manual_rc_interval_s"];rc.steering_us=1700. if lo<=t<hi else 1500.
        events.append((ns,prefix+"/rc/input",rc))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"),storage_id="sqlite3"),rosbag2_py.ConverterOptions("cdr","cdr"))
    for topic,type_name in types.items():writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=type_name,serialization_format="cdr"))
    for ns,topic,message in sorted(events,key=lambda e:(e[0],e[1])):writer.write(topic,serialize_message(message),ns)
    del writer
    return {"kind":fixture["kind"],"messages":len(events),"topics":types}


def subscriptions(record):
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    def pose_values(p):
        return {"pose_values":[p.pose.position.x,p.pose.position.y,p.pose.position.z,p.pose.orientation.x,
            p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w],"covariance":list(p.covariance)}
    return [(AckermannDriveStamped,"control/autonomy/ackermann",lambda m:record("ackermann",m,
                {"speed":m.drive.speed,"steering":m.drive.steering_angle})),
            (Odometry,"state/odometry",lambda m:record("odometry",m,{"body":m.child_frame_id,**pose_values(m.pose)})),
            (PoseWithCovarianceStamped,"state/pose",lambda m:record("pose",m,pose_values(m.pose)))]


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind,lo,hi):
        found=[e for e in events if e["kind"]==kind and lo<=e["t_s"]<hi]
        if not found:raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return found
    commands=rows("ackermann",1.,5.)
    if not all(0<e["speed"]<=6.+1e-6 and abs(e["steering"])<1.2 for e in commands):
        raise RuntimeError("Waypoint target left fixture domain")
    if not any(e["steering"]>.01 for e in commands) or not any(e["steering"]<-.01 for e in commands):
        raise RuntimeError("Pose/reference did not exercise both steering signs")
    if not all(e["mode"]==SelectedControl.MANUAL for e in rows("selection",3.75,3.95)):
        raise RuntimeError("RC did not take full priority")
    if not all(e["erpm"]==0 for e in rows("motor",3.75,3.95)) or not all(abs(e["servo"]-.6)<1e-6 for e in rows("servo",3.75,3.95)):
        raise RuntimeError("Manual pair mixed with autonomy")
    if not all(e["mode"]==SelectedControl.AUTONOMY for e in rows("selection",4.2,4.5)):
        raise RuntimeError("Valid neutral RC did not resume autonomy")
    for lo,hi in ((5.35,5.55),(5.85,6.15),(7.5,7.9)):
        if not all(e["erpm"]==0 for e in rows("motor",lo,hi)) or not all(e["mode"]==SelectedControl.STOP for e in rows("selection",lo,hi)):
            raise RuntimeError("Body/age/silence did not expire the pose chain")
    if not all(e["erpm"]>0 for e in rows("motor",6.55,6.9)):
        raise RuntimeError("Valid source did not recover the pose chain")
    # Repeated acquisition stamps may carry later stale payloads. Keep the
    # actual fresh candidates, rather than overwriting the original measure.
    origin=json.loads((share()/"data/waypoint_pp_replay.json").read_text())["origin_ros_s"]
    odom={}
    for event in events:
        if event["kind"]!="odometry" or event["body"]!="fixture_rear_axle_flu":continue
        age=origin+event["t_s"]-event["stamp_ns"]*1e-9
        if -.02<=age<.2:odom.setdefault(event["stamp_ns"],[]).append(event)
    poses={e["stamp_ns"]:e for e in events if e["kind"]=="pose"}
    source={e["stamp_ns"]:e for e in events if e["kind"]=="ackermann"}
    for stamp,pose in poses.items():
        originals=odom.get(stamp,[])
        if not any(pose["frame"]==original["frame"] and pose["pose_values"]==original["pose_values"]
                   and pose["covariance"]==original["covariance"] for original in originals):
            raise RuntimeError("Pose projection changed original measurement")
    for stamp,command in source.items():
        if stamp not in poses or poses[stamp]["frame"]!="fixture_map" or command["frame"]!="fixture_rear_axle_flu":
            raise RuntimeError("Waypoint source stamp/body contract lost")
    matched=0
    for event in (e for e in events if e["kind"]=="actuator"):
        command=source.get(event["stamp_ns"])
        if command is None:continue
        if event["frame"]!=command["frame"] or abs(event["erpm"]-100*command["speed"])>1e-4 or abs(event["servo"]-(.5-.08*command["steering"]))>1e-6:
            raise RuntimeError("Waypoint fixture conversion changed")
        matched+=1
    if matched<80:raise RuntimeError("Insufficient original-stamp conversions")
    return {"waypoint_target_domain":True,"both_steering_signs":True,"exact_pose_projection":True,
        "original_stamp_body_and_map":True,"rc_priority":True,"neutral_resume":True,"wrong_body_expiry":True,
        "stale_input_expiry":True,"silence_expiry":True,"fresh_input_recovery":True,"matched_conversions":matched}


def run(output):
    from .software_replay import run_campaign
    fixture=json.loads((share()/"data/waypoint_pp_replay.json").read_text())
    controller={"launch":"waypoint_pp_software.launch.py","subscriptions":subscriptions,
        "expected_subscribers":{"state/odometry":{"odometry_pose","control_replay_observer"},
            "state/pose":{"waypoint_pp","control_replay_observer"},
            "control/autonomy/ackermann":{"ackermann_adapter","control_replay_observer"}},
        "clock_kinds":["selection","backend_status"]}
    return run_campaign(output,inspect_profile(),fixture,make_bag,verify,controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv);args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve());print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1


if __name__=="__main__":raise SystemExit(main())
