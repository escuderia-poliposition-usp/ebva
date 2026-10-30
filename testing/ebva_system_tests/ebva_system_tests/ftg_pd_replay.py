"""Finite synthetic two-input FTG/PD campaign, without drivers or calibration."""
import argparse,json,math
from pathlib import Path
from .resources import share, fixture_data, fixture_path, profile_path
from .ftg_pd_software import NAMESPACE,inspect_profile


def make_bag(output, fixture):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import LaserScan,JointState
    from ebva_control_msgs.msg import RcInput
    prefix="/"+NAMESPACE;origin=round(fixture["origin_ros_s"]*1e9);events=[]
    types={prefix+"/sensors/scan":"sensor_msgs/msg/LaserScan",prefix+"/sensors/steering":"sensor_msgs/msg/JointState",prefix+"/rc/input":"ebva_control_msgs/msg/RcInput"}
    for i in range(round(fixture["duration_s"]/fixture["scan_period_s"])+1):
        t=round(i*fixture["scan_period_s"],6);ns=origin+round(t*1e9)
        p=next((p for p in fixture["phases"] if p["start"]<=t<p["end"]),None)
        if p is None or p["mode"]=="silence":continue
        if p["mode"]!="scan_silence":
            scan=LaserScan();scan.header.stamp.sec,scan.header.stamp.nanosec=divmod(ns,1_000_000_000)
            scan.header.frame_id="fixture_control_flu";scan.angle_min=-3*math.pi/4;scan.angle_max=3*math.pi/4
            scan.angle_increment=math.pi/720;scan.range_min=.01;scan.range_max=10.;scan.scan_time=fixture["scan_period_s"]
            scan.ranges=[2.+math.exp(-.5*((-3*math.pi/4+j*math.pi/720-math.radians(p["goal_deg"]))/.3)**2) for j in range(1081)]
            if p["mode"]=="large_gap":scan.ranges=[10.]*1081
            if p["mode"]=="bad_scan":scan.ranges[100]=float('inf')
            events.append((ns,prefix+"/sensors/scan",scan))
        if p["mode"]!="feedback_silence":
            joint=JointState();stamp=ns-800_000_000 if p["mode"]=="stale_feedback" else ns
            joint.header.stamp.sec,joint.header.stamp.nanosec=divmod(stamp,1_000_000_000)
            joint.header.frame_id="fixture_control_flu";joint.name=["virtual_steering"];joint.position=[p["feedback"]]
            events.append((ns,prefix+"/sensors/steering",joint))
    for i in range(round(fixture["duration_s"]/fixture["rc_period_s"])+1):
        t=round(i*fixture["rc_period_s"],6);ns=origin+round(t*1e9);rc=RcInput()
        rc.header.stamp.sec,rc.header.stamp.nanosec=divmod(ns,1_000_000_000);rc.header.frame_id="fixture_rc"
        rc.sequence=i+1;rc.valid=True;rc.rf_state=RcInput.RF_UNKNOWN;rc.reason="synthetic_replay";rc.throttle_us=1500.
        lo,hi=fixture["manual_rc_interval_s"];rc.steering_us=1700. if lo<=t<hi else 1500.
        events.append((ns,prefix+"/rc/input",rc))
    writer=rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/"input_bag"),storage_id="sqlite3"),rosbag2_py.ConverterOptions("cdr","cdr"))
    for topic,typename in types.items():writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typename,serialization_format="cdr"))
    for ns,topic,message in sorted(events,key=lambda e:(e[0],e[1])):writer.write(topic,serialize_message(message),ns)
    del writer
    return {"kind":fixture["kind"],"messages":len(events),"topics":types}


def subscriptions(record):
    from ebva_control_msgs.msg import FtgPdCommand
    from sensor_msgs.msg import LaserScan,JointState
    from diagnostic_msgs.msg import DiagnosticArray
    def command(m):
        record("ftg",m,{"contract":m.contract,"throttle":m.throttle,"pd_effort":m.steering_effort,
            "feedback_stamp_ns":m.feedback_header.stamp.sec*1_000_000_000+m.feedback_header.stamp.nanosec,
            "feedback_frame":m.feedback_header.frame_id})
    return [(FtgPdCommand,"control/autonomy/ftg_pd",command),
        (LaserScan,"sensors/scan",lambda m:record("scan",m,{})),
        (JointState,"sensors/steering",lambda m:record("feedback",m,{"value":m.position[0]})),
        (DiagnosticArray,"control/ftg_status",lambda m:record("ftg_status",m,{"reason":m.status[0].message if m.status else ""})),
        (DiagnosticArray,"control/ftg_adapter_status",lambda m:record("ftg_adapter_status",m,{}))]


def verify(events):
    from ebva_control_msgs.msg import SelectedControl
    def rows(kind,lo,hi):
        found=[e for e in events if e["kind"]==kind and lo<=e["t_s"]<hi]
        if not found:raise RuntimeError(f"No {kind} in {lo}..{hi}")
        return found
    if not all(abs(e["throttle"]-.1)<1e-5 and abs(e["pd_effort"])<1e-4 for e in rows("ftg",.8,1.3)):
        raise RuntimeError("FTG straight/steady output differs from original law")
    if not all(e["pd_effort"]>.05 for e in rows("ftg",1.8,2.2)) or not all(e["pd_effort"]<-.05 for e in rows("ftg",2.8,3.2)):
        raise RuntimeError("Mirrored FTG/feedback response differs")
    if not all(e["mode"]==SelectedControl.MANUAL for e in rows("selection",3.75,3.95)):
        raise RuntimeError("RC did not override FTG pair")
    if not all(e["erpm"]==0 for e in rows("motor",3.75,3.95)) or not all(abs(e["servo"]-.6)<1e-6 for e in rows("servo",3.75,3.95)):
        raise RuntimeError("Manual pair mixed with FTG output")
    if not all(e["mode"]==SelectedControl.AUTONOMY for e in rows("selection",4.15,4.35)):
        raise RuntimeError("Neutral RC did not resume valid FTG")
    for lo,hi in [(4.9,5.2),(6.5,6.75),(7.,7.5),(7.9,8.1),(9.5,9.9)]:
        if not all(e["erpm"]==0 for e in rows("motor",lo,hi)):
            raise RuntimeError("Lost/invalid primary or feedback input did not expire pair")
    for lo,hi in [(5.65,5.9),(8.6,8.9)]:
        if not all(e["erpm"]>0 for e in rows("motor",lo,hi)):
            raise RuntimeError("Fresh two-input recovery failed")
    if not all(abs(e["throttle"]-.12)<1e-6 for e in rows("ftg",5.7,5.9)):
        raise RuntimeError("Large-gap branch did not recover with original throttle cap")
    if not any(e["reason"]=="invalid_scan" for e in rows("ftg_status",7.,7.5)):
        raise RuntimeError("Invalid scan not diagnosed")
    scans={e["stamp_ns"] for e in events if e["kind"]=="scan"}
    feedbacks={e["stamp_ns"] for e in events if e["kind"]=="feedback"}
    sources={}
    for e in (e for e in events if e["kind"]=="ftg"):
        sources.setdefault(min(e["stamp_ns"],e["feedback_stamp_ns"]),[]).append(e)
    matched=0
    for e in (e for e in events if e["kind"]=="actuator"):
        candidates=sources.get(e["stamp_ns"],[])
        if not candidates:continue
        s=next((s for s in candidates if abs(e["erpm"]-1000*s["throttle"])<=1e-6
                and abs(e["servo"]-max(.4,min(.6,.5-.1*s["pd_effort"])))<=1e-6),None)
        if s is None:raise RuntimeError("FTG-specific fixture map mismatch")
        if (s["contract"]!="ftg_pd_sample_v1" or s["stamp_ns"] not in scans or s["feedback_stamp_ns"] not in feedbacks
                or e["frame"]!=s["frame"] or e["frame"]!=s["feedback_frame"] or e["frame"]!="fixture_control_flu"):
            raise RuntimeError("Input provenance, oldest source stamp or frame lost")
        if abs(e["erpm"]-1000*s["throttle"])>1e-6 or abs(e["servo"]-max(.4,min(.6,.5-.1*s["pd_effort"])))>1e-6:
            raise RuntimeError("FTG-specific fixture map mismatch")
        matched+=1
    if matched<100:raise RuntimeError("Insufficient matched dual-input conversions")
    return {"straight_and_throttle":True,"mirrored_pd_feedback":True,"large_gap_branch":True,"rc_priority":True,"neutral_resume":True,
        "feedback_silence_expiry":True,"scan_silence_expiry":True,"invalid_scan_expiry":True,
        "stale_feedback_expiry":True,"fresh_pair_recovery":True,"oldest_input_stamp_and_provenance":True,
        "matched_conversions":matched}


def run(output):
    from .software_replay import run_campaign
    fixture=json.loads((share()/"data/ftg_pd_replay.json").read_text())
    controller={"launch":"ftg_pd_software.launch.py","subscriptions":subscriptions,
        "expected_subscribers":{"sensors/scan":{"ftg_pd","control_replay_observer"},"sensors/steering":{"ftg_pd","control_replay_observer"},
            "control/autonomy/ftg_pd":{"ftg_pd_adapter","control_replay_observer"},"control/ftg_status":{"control_replay_observer"},
            "control/ftg_adapter_status":{"control_replay_observer"}},
        "clock_kinds":["ftg_status","ftg_adapter_status","selection","backend_status"]}
    return run_campaign(output,inspect_profile(),fixture,make_bag,verify,controller)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv);args.output.mkdir(parents=True,exist_ok=False)
    result=run(args.output.resolve());print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1


if __name__=="__main__":raise SystemExit(main())
