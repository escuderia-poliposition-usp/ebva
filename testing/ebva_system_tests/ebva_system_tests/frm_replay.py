"""Finite synthetic scan replay of the FRM point-sample software chain."""
import argparse
import json
import math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .frm_control_software import inspect_profile, NAMESPACE


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan
    from ebva_control_msgs.msg import RcInput
    prefix="/"+NAMESPACE
    types={prefix+"/sensors/scan":"sensor_msgs/msg/LaserScan",prefix+"/rc/input":"ebva_control_msgs/msg/RcInput"}
    events=[];origin=round(fixture["origin_ros_s"]*1e9)
    for i in range(round(fixture["duration_s"]/fixture["scan_period_s"])+1):
        t=round(i*fixture["scan_period_s"],6)
        p=next((p for p in fixture["phases"] if p["start"]<=t<p["end"]),None)
        if p is None or p["mode"]=="silence":continue
        scan=LaserScan();ns=origin+round(t*1e9)
        stamp=ns-round(.8e9) if p["mode"]=="stale" else ns
        scan.header.stamp.sec,scan.header.stamp.nanosec=divmod(stamp,1_000_000_000)
        scan.header.frame_id="fixture_lidar_flu";scan.angle_min=-3*math.pi/4
        scan.angle_increment=math.pi/720;scan.angle_max=scan.angle_min+1079*scan.angle_increment
        scan.range_min=.001;scan.range_max=20.;scan.scan_time=fixture["scan_period_s"]
        ranges=[1.]*1080;ranges[540]=float("nan") if p["mode"]=="nan" else p["front"]
        ranges[900]=1.+max(0.,p["lateral"]);ranges[180]=1.+max(0.,-p["lateral"])
        scan.ranges=ranges;events.append((ns,prefix+"/sensors/scan",scan))
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
    from ebva_control_msgs.msg import DriveEffort
    from diagnostic_msgs.msg import DiagnosticArray
    return [(DriveEffort,"control/autonomy/effort",lambda m:record("effort",m,
                {"contract":m.contract,"longitudinal":m.longitudinal,"steering":m.steering})),
            (DiagnosticArray,"control/frm_status",lambda m:record("frm_status",m,
                {"reason":m.status[0].message if m.status else ""})),
            (DiagnosticArray,"control/effort_status",lambda m:record("effort_status",m,{}))]


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind,lo,hi):
        found=[e for e in events if e["kind"]==kind and lo<=e["t_s"]<hi]
        if not found:raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return found
    if not all(abs(e["longitudinal"]-.15)<1e-6 and abs(e["steering"])<1e-6 for e in rows("effort",1.,1.4)):
        raise RuntimeError("Steady FRM effort differs from original law")
    for lo,hi,value in [(1.8,2.2,.03),(2.8,3.2,-.03)]:
        if not all(abs(e["steering"]-value)<1e-6 for e in rows("effort",lo,hi)):
            raise RuntimeError("Mirrored effort sign/gain differs from original law")
    if not any(e["longitudinal"]<-.09 for e in rows("effort",4.95,5.15)):
        raise RuntimeError("Original negative sample-difference transient lost")
    if not all(e["mode"]==SelectedControl.MANUAL for e in rows("selection",3.75,3.95)):
        raise RuntimeError("RC did not take full priority")
    if not all(e["erpm"]==0 for e in rows("motor",3.75,3.95)) or not all(abs(e["servo"]-.6)<1e-6 for e in rows("servo",3.75,3.95)):
        raise RuntimeError("Manual pair mixed with autonomous effort")
    if not all(e["mode"]==SelectedControl.AUTONOMY for e in rows("selection",4.2,4.5)):
        raise RuntimeError("Valid neutral RC did not resume autonomy")
    for lo,hi in [(5.8,5.95),(6.15,6.35),(7.5,7.9)]:
        if not all(e["erpm"]==0 for e in rows("motor",lo,hi)):
            raise RuntimeError("Invalid/stale/silent scan did not expire autonomous effort")
    if not any(e["mode"]==SelectedControl.STOP for e in rows("selection",5.8,6.35)):
        raise RuntimeError("Expiry did not reach the authority")
    if not all(e["erpm"]>0 for e in rows("motor",6.7,7.)):
        raise RuntimeError("Fresh scan did not recover after invalid interval")
    if not any(e["reason"]=="invalid_consumed_range" for e in rows("frm_status",5.55,5.95)):
        raise RuntimeError("Invalid scan was not diagnosed")
    if not any(e["reason"]=="stale_or_future" for e in rows("frm_status",6.15,6.35)):
        raise RuntimeError("Stale scan was not diagnosed")
    source={e["stamp_ns"]:e for e in events if e["kind"]=="effort"}
    matched=0
    for e in (e for e in events if e["kind"]=="actuator"):
        s=source.get(e["stamp_ns"])
        if s is None:continue
        if s["contract"]!="frm_point_sample_v1" or e["frame"]!=s["frame"] or e["frame"]!="fixture_lidar_flu":
            raise RuntimeError("Atomic contract or source frame lost")
        if abs(e["erpm"]-100*s["longitudinal"])>1e-6 or abs(e["servo"]-(.5-.1*s["steering"]))>1e-6:
            raise RuntimeError("Artificial effort mapping differs from explicit gains")
        matched+=1
    if matched<80:raise RuntimeError("Insufficient matched original stamps")
    return {"steady_longitudinal":True,"sample_difference_negative_transient":True,"mirrored_effort":True,
            "atomic_pair_header_and_conversion":True,"rc_priority":True,"neutral_resume":True,
            "invalid_scan_expiry":True,"stale_scan_expiry":True,"silence_expiry":True,
            "fresh_input_recovery":True,"matched_conversions":matched}


def run(output):
    from .software_replay import run_campaign
    fixture=json.loads((share()/"data/frm_control_replay.json").read_text())
    controller={"launch":"frm_control_software.launch.py","subscriptions":subscriptions,
        "expected_subscribers":{"sensors/scan":{"frm_point_sample"},
            "control/autonomy/effort":{"frm_effort_adapter","control_replay_observer"},
            "control/frm_status":{"control_replay_observer"},"control/effort_status":{"control_replay_observer"}},
        "clock_kinds":["frm_status","effort_status","selection","backend_status"]}
    return run_campaign(output,inspect_profile(),fixture,make_bag,verify,controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv);args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve());print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1


if __name__=="__main__":raise SystemExit(main())
