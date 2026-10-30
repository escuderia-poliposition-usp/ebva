"""Finite synthetic rosbag2 replay of the isolated software control profile.

Imports are safe. The CLI creates a new output directory, synthetic input bag,
four software ROS processes, a player and an observer. It never starts a driver.
"""
import argparse
import json
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .lane_control_software import NAMESPACE, inspect_profile


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import Point
    from std_msgs.msg import Empty
    from lane_msgs.msg import LaneBoundaries
    from ebva_control_msgs.msg import RcInput
    prefix="/"+NAMESPACE
    types={prefix+"/perception/lanes":"lane_msgs/msg/LaneBoundaries",
           prefix+"/rc/input":"ebva_control_msgs/msg/RcInput",
           prefix+"/control/lane_reset":"std_msgs/msg/Empty"}
    events=[]
    origin=int(fixture["origin_ros_s"]*1e9)
    duration=fixture["duration_s"]
    def stamp(message,t):
        ns=origin+round(t*1e9)
        message.header.stamp.sec,message.header.stamp.nanosec=divmod(ns,1_000_000_000)
        return ns
    for i in range(round(duration/fixture["lane_period_s"])+1):
        t=round(i*fixture["lane_period_s"],6)
        phase=next((p for p in fixture["lanes"] if p["start"]<=t<p["end"]),None)
        if phase is None or phase["mode"]=="silence":
            continue
        message=LaneBoundaries();ns=stamp(message,t)
        message.header.frame_id="fixture_ground_frd"
        if phase["mode"]!="empty":
            message.left_lane=[Point(x=x,y=phase["offset"]-.25,z=0.) for x in (.1,.3,.5,.7,1.)]
            if phase["mode"]=="both":
                message.right_lane=[Point(x=x,y=phase["offset"]+.25,z=0.) for x in (.1,.3,.5,.7,1.)]
        events.append((ns,prefix+"/perception/lanes",message))
    for i in range(round(duration/fixture["rc_period_s"])+1):
        t=round(i*fixture["rc_period_s"],6)
        message=RcInput();ns=stamp(message,t)
        message.header.frame_id="fixture_rc";message.sequence=i+1
        message.valid=True;message.rf_state=RcInput.RF_UNKNOWN;message.reason="synthetic_replay"
        message.throttle_us=1500.
        lo,hi=fixture["manual_rc_interval_s"]
        message.steering_us=1700. if lo<=t<hi else 1500.
        events.append((ns,prefix+"/rc/input",message))
    events.append((origin+round(fixture["reset_s"]*1e9),prefix+"/control/lane_reset",Empty()))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"),storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("cdr","cdr"))
    for topic,type_name in types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=type_name,serialization_format="cdr"))
    for ns,topic,message in sorted(events,key=lambda event:(event[0],event[1])):
        writer.write(topic,serialize_message(message),ns)
    del writer
    return {"kind":fixture["kind"],"messages":len(events),"topics":types}


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind,lo,hi):
        found=[e for e in events if e["kind"]==kind and lo<=e["t_s"]<hi]
        if not found:
            raise RuntimeError(f"No observed {kind} samples in {lo}..{hi}")
        return found
    straight=rows("ackermann",1.,1.4)
    if not all(e["speed_mps"]>0 and abs(e["steering_rad"])<1e-6 for e in straight):
        raise RuntimeError("Straight lane did not produce positive speed/zero angle")
    right=rows("ackermann",1.8,2.2);left=rows("ackermann",2.6,3.)
    if not all(e["steering_rad"]<-.01 for e in right) or not all(e["steering_rad"]>.01 for e in left):
        raise RuntimeError("Ground FRD / Ackermann FLU steering sign mismatch")
    if not any(e["mode"]==-1. for e in rows("lane_debug",3.15,3.35)):
        raise RuntimeError("One-line fallback was not observed")
    if not all(e["mode"]==SelectedControl.MANUAL for e in rows("selection",3.7,3.95)):
        raise RuntimeError("RC did not override the controller")
    if not all(e["erpm"]==0 for e in rows("motor",3.7,3.95)):
        raise RuntimeError("Autonomous throttle leaked through steering-only RC override")
    if not all(abs(e["servo"]-.6)<1e-9 for e in rows("servo",3.7,3.95)):
        raise RuntimeError("Manual steering did not reach the same backend")
    if not all(e["mode"]==SelectedControl.AUTONOMY for e in rows("selection",4.15,4.4)):
        raise RuntimeError("Neutral RC did not restore valid autonomous control")
    for lo,hi in ((5.1,5.25),(5.4,5.65),(7.,7.4)):
        if not all(e["erpm"]==0 for e in rows("motor",lo,hi)):
            raise RuntimeError(f"Loss/latch/watchdog did not stop in {lo}..{hi}")
    if not any(e["erpm"]>0 for e in rows("motor",6.15,6.5)):
        raise RuntimeError("Explicit lane reset did not recover tracking")
    ack={e["stamp_ns"]:e for e in events if e["kind"]=="ackermann"}
    matches=0
    for e in (e for e in events if e["kind"]=="actuator"):
        source=ack.get(e["stamp_ns"])
        if source is None:
            continue
        if e["frame"]!=source["frame"] or e["frame"]!="fixture_rear_axle_flu":
            raise RuntimeError("Adapter changed the reference frame")
        erpm=max(-600.,min(600.,100.*max(-2.,min(2.,source["speed_mps"]))))
        servo=max(.4,min(.6,.5-.2*max(-.5,min(.5,source["steering_rad"]))))
        if abs(e["erpm"]-erpm)>1e-6 or abs(e["servo"]-servo)>1e-6:
            raise RuntimeError("Fixture conversion mismatch")
        matches+=1
    if matches<50:
        raise RuntimeError("Insufficient matched source stamps to verify conversion")
    return {"straight":True,"mirrored_steering":True,"one_lane":True,"rc_override":True,
            "neutral_resume":True,"line_loss_latch":True,"explicit_reset":True,
            "input_silence_watchdog":True,"source_stamp_and_frame_preserved":True,
            "matched_conversions":matches}


def subscriptions(record):
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Float32MultiArray
    return [(AckermannDriveStamped,"control/autonomy/ackermann",lambda m:record("ackermann",m,
                {"speed_mps":m.drive.speed,"steering_rad":m.drive.steering_angle})),
            (Float32MultiArray,"debug/lane",lambda m:record("lane_debug",m,{"mode":m.data[9],"latched":m.data[8]})),
            (DiagnosticArray,"control/ackermann_status",lambda m:record("ackermann_status",m,
                {"states":[{"name":s.name,"reason":s.message,
                            "values":{v.key:v.value for v in s.values}} for s in m.status]}))]


def run(output):
    from .software_replay import run_campaign
    fixture=json.loads((share()/"data/lane_control_replay.json").read_text())
    controller={"launch":"lane_control_software.launch.py", "subscriptions":subscriptions,
        "expected_subscribers":{"perception/lanes":{"lane_keeping_node"},
            "control/lane_reset":{"lane_keeping_node"},
            "control/autonomy/ackermann":{"ackermann_adapter","control_replay_observer"},
            "debug/lane":{"control_replay_observer"},
            "control/ackermann_status":{"control_replay_observer"}},
        "clock_kinds":["ackermann","actuator","selection","backend_status"]}
    return run_campaign(output,inspect_profile(),fixture,make_bag,verify,controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve())
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
