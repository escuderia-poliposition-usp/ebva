"""Existing deterministic software campaigns; excludes hardware probes."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
CAMPAIGNS=[
 ("supervisor","supervisor_replay",[],"supervisor_software"),
 ("manual","manual_core_replay",[],"manual_core"),
 ("lane","lane_replay",[],"lane_control_software"),
 ("frm","frm_replay",[],"frm_control_software"),
 ("ftg_pd","ftg_pd_replay",[],"ftg_pd_software"),
 ("csv_frenet","csv_frenet_replay",[],"csv_frenet_software"),
 ("waypoint_pp","waypoint_pp_replay",[],"waypoint_pp_software"),
 ("waypoint_control","effort_steering_replay",["--method","waypoint_control"],"waypoint_control_software"),
 ("lidar_frenet_pp","effort_steering_replay",["--method","lidar_frenet_pp"],"lidar_frenet_pp_software"),
 ("cbf_yaw_rate","cbf_yaw_rate_replay",[],"cbf_yaw_rate_software"),
 ("cbf_yaw_rate_margin","alternate_cbf_replay",["--model","yaw_rate_margin"],"cbf_yaw_rate_margin_software"),
 ("cbf_steering_tangent","alternate_cbf_replay",["--model","steering_tangent"],"cbf_steering_tangent_software"),
 ("stanley_frenet","stanley_frenet_replay",[],"stanley_frenet_software"),
 ("cbf_pipeline","cbf_pipeline_replay",[],None),
 ("imu_prior","imu_prior_replay",[],None),
 ("lane_perception","lane_perception_replay",[],None)]
FORBIDDEN={"rc_acquisition","vesc_driver_node","urg_node2_node","yolo_node","component_container_isolated"}
def validate_campaign_profile(name):
    from .resources import profile_path
    from ebva_bringup.profiles import inspect_profile
    report=inspect_profile(profile_path(name))
    if report["launches"] or any(n["executable"] in FORBIDDEN for n in report["nodes"]):
        raise ValueError("A software campaign cannot include device providers")
    return report
def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--campaign",action="append",choices=[r[0] for r in CAMPAIGNS])
    args=parser.parse_args(argv)
    if os.environ.get("ROS_LOCALHOST_ONLY")!="1" or not os.environ.get("ROS_DOMAIN_ID"):
        parser.error("Use ROS_LOCALHOST_ONLY=1 and an explicit isolated ROS_DOMAIN_ID")
    if args.output.exists():parser.error("Output must be a new generated-results directory")
    args.output.mkdir(parents=True)
    results=[]
    for name,module,extra,profile in CAMPAIGNS:
        if args.campaign and name not in args.campaign:continue
        if profile:validate_campaign_profile(profile)
        output=args.output/(name+".json" if name=="cbf_pipeline" else name)
        command=[sys.executable,"-m","ebva_system_tests."+module,"--output",str(output),*extra]
        started=time.monotonic()
        with (args.output/(name+".log")).open("w") as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=process.wait(timeout=90)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
                code=124
        results.append({"campaign":name,"exit_code":code,"duration_s":round(time.monotonic()-started,3),"command":command})
        (args.output/"results.json").write_text(json.dumps(results,indent=2))
        print(name,"passed" if code==0 else "FAILED",flush=True)
        time.sleep(.3)
    return int(any(row["exit_code"] for row in results))
if __name__=="__main__":raise SystemExit(main())
