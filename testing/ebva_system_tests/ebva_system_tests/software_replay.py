from .resources import profile_path
"""Finite isolated rosbag2 orchestration shared by C1 and C2 consumers."""
import json
import os
import signal
import subprocess
import time


def run_campaign(output, profile, fixture, make_bag, verify, controller):
    NAMESPACE=profile["namespace"]
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import Float64, Float32MultiArray
    from std_srvs.srv import SetBool
    from rosbag2_interfaces.srv import Resume, PlayNext
    from diagnostic_msgs.msg import DiagnosticArray
    from ackermann_msgs.msg import AckermannDriveStamped
    from ebva_control_msgs.msg import RcInput, ActuatorCommand, SelectedControl
    result={"passed":False,"kind":"synthetic_rosbag2_replay","physical_qualification":False,
            "profile":profile,"calibration_qualification":"software_fixture",
            "domain_id":os.environ.get("ROS_DOMAIN_ID"),"localhost_only":os.environ.get("ROS_LOCALHOST_ONLY"),
            "image":os.environ.get("VALIDATION_IMAGE",""),"devices_started":[]}
    result["bag"]=make_bag(output,fixture)
    events=[];trace=(output/"trace.jsonl").open("x")
    rclpy.init()
    node=Node("control_replay_observer",namespace=NAMESPACE,
              parameter_overrides=[Parameter("use_sim_time",value=True)])
    rc_latest=None
    def record(kind, message, values):
        item={"kind":kind,"t_s":node.get_clock().now().nanoseconds/1e9-fixture["origin_ros_s"],**values}
        if hasattr(message,"header"):
            item.update(stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
                        frame=message.header.frame_id)
        events.append(item);trace.write(json.dumps(item)+"\n")
    def rc(message):
        nonlocal rc_latest
        rc_latest=message
        record("rc",message,{"valid":message.valid,"steering":message.steering_us})
    subscriptions=[
        (RcInput,"rc/input",rc),
        (ActuatorCommand,"control/autonomy/actuators",lambda m:record("actuator",m,{"erpm":m.erpm,"servo":m.servo})),
        (SelectedControl,"control/selected",lambda m:record("selection",m,{"mode":m.mode,"reason":m.reason})),
        (DiagnosticArray,"control/backend_status",lambda m:record("backend_status",m,{})),
        (Float64,"commands/motor/speed",lambda m:record("motor",m,{"erpm":m.data})),
        (Float64,"commands/servo/position",lambda m:record("servo",m,{"servo":m.data})),
    ]
    subscriptions += controller["subscriptions"](record)
    for type_name,topic,callback in subscriptions:
        node.create_subscription(type_name,topic,callback,10)
    def service(name,value,timeout=.7):
        client=node.create_client(SetBool,name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return False
            future=client.call_async(SetBool.Request(data=value))
            deadline=time.monotonic()+timeout
            while not future.done() and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.005)
            return future.done() and future.result().success
        finally:node.destroy_client(client)
    children=[];logs=[];armed=False
    commands=[
        ["ros2","launch","ebva_bringup","bringup.launch.py","profile:="+profile_path(profile["profile"])],
        ["ros2","bag","play",str(output/"input_bag"),"--clock","100","--start-paused","--disable-keyboard-controls"]]
    result["commands"]=commands
    try:
        launch_log=(output/"launch.log").open("w");logs.append(launch_log)
        launch=subprocess.Popen(commands[0],stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(launch)
        ready=time.monotonic()+10.
        while time.monotonic()<ready:
            client=node.create_client(SetBool,"control/set_autonomy_enabled")
            available=client.wait_for_service(timeout_sec=.1);node.destroy_client(client)
            if available:
                break
            if launch.poll() is not None:
                raise RuntimeError("Software profile exited before service discovery")
        else:raise RuntimeError("Authority service unavailable")
        player_log=(output/"player.log").open("w");logs.append(player_log)
        player=subprocess.Popen(commands[1],stdout=player_log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(player)
        # Discover every pipeline endpoint before releasing bag time. On ARM,
        # authority services can appear seconds before the lane/clock graph.
        expected={"rc/input":{"command_authority","control_replay_observer"},
                  "control/autonomy/actuators":{"command_authority","control_replay_observer"},
                  "control/selected":{"vesc_backend","control_replay_observer"},
                  "commands/motor/speed":{"control_replay_observer"},
                  "commands/servo/position":{"control_replay_observer"},
                  "control/backend_status":{"control_replay_observer"},
                  **controller["expected_subscribers"]}
        resume=node.create_client(Resume,"/rosbag2_player/resume")
        try:
            ready=time.monotonic()+12.
            while time.monotonic()<ready:
                rclpy.spin_once(node,timeout_sec=.01)
                matched=all(names <= {info.node_name for info in node.get_subscriptions_info_by_topic(
                    "/"+NAMESPACE+"/"+topic)} for topic,names in expected.items())
                clock_names={info.node_name for info in node.get_subscriptions_info_by_topic("/clock")}
                matched=matched and ({spec[2] for spec in profile["nodes"]}|{"control_replay_observer"}) <= clock_names
                result["startup_graph"]={"clock_subscribers":sorted(clock_names),
                    "subscriptions":{topic:sorted(info.node_name for info in node.get_subscriptions_info_by_topic(
                        "/"+NAMESPACE+"/"+topic)) for topic in expected},
                    "resume_service_ready":resume.service_is_ready()}
                if matched and resume.service_is_ready():
                    # All nodes are constructed; let DDS establish the player
                    # matches before its first volatile lane/RC samples.
                    settled=time.monotonic()+.5
                    while time.monotonic()<settled:rclpy.spin_once(node,timeout_sec=.005)
                    # One paused step primes /clock. Graph discovery alone
                    # does not prove that every node has received ROS time.
                    step=node.create_client(PlayNext,"/rosbag2_player/play_next")
                    try:
                        if not step.wait_for_service(timeout_sec=2.):
                            raise RuntimeError("Paused player step service unavailable")
                        first=step.call_async(PlayNext.Request())
                        end=time.monotonic()+3.
                        while not first.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
                        if not first.done() or not first.result().success:
                            raise RuntimeError("Paused player could not prime ROS time")
                    finally:node.destroy_client(step)
                    origin_ns=round(fixture["origin_ros_s"]*1e9)
                    end=time.monotonic()+8.
                    while time.monotonic()<end:
                        rclpy.spin_once(node,timeout_sec=.005)
                        synced={e["kind"] for e in events if e.get("stamp_ns",0)>=origin_ns}
                        if set(controller["clock_kinds"]) <= synced:
                            result["startup_clocks_synchronised"]=True
                            break
                        if launch.poll() is not None or player.poll() is not None:
                            raise RuntimeError("Process exited while priming ROS time")
                    else:raise RuntimeError("Software nodes did not acknowledge initial ROS time")
                    future=resume.call_async(Resume.Request())
                    end=time.monotonic()+2.
                    while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
                    if not future.done() or future.exception() is not None:
                        raise RuntimeError("Paused rosbag2 player did not resume")
                    result["startup_graph_ready"]=True
                    break
                if launch.poll() is not None or player.poll() is not None:
                    raise RuntimeError("Software process exited during replay readiness")
            else:raise RuntimeError("Incomplete software replay graph before resume")
        finally:node.destroy_client(resume)
        deadline=time.monotonic()+18.
        next_arm=0.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.005)
            if launch.poll() is not None:
                raise RuntimeError("A software profile process exited")
            if not armed and rc_latest is not None and rc_latest.valid and time.monotonic()>=next_arm:
                next_arm=time.monotonic()+.15
                if service("control/set_autonomy_enabled",True):
                    armed=service("control/set_actuation_enabled",True)
            if player.poll() is not None:
                if player.returncode:
                    raise RuntimeError("rosbag2 player failed")
                end=time.monotonic()+.5
                while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
                break
        else:raise RuntimeError("Finite replay deadline exceeded")
        if not armed:
            raise RuntimeError("Software backend never armed from valid neutral replay RC")
        result["scenarios"]=verify(events);result["passed"]=True
    except Exception as exc:
        result["error"]=repr(exc)
    finally:
        result["cleanup_disarmed"] = service("control/set_actuation_enabled",False)
        service("control/set_autonomy_enabled",False)
        forced=False
        for process in reversed(children):
            if process.poll() is None:
                # ros2 launch forwards this signal to its children. Signalling
                # its whole group first would deliver SIGINT twice to nodes.
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=7.)
                except subprocess.TimeoutExpired:
                    forced=True
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
        for log in logs:log.close()
        result["child_returncodes"]=[process.returncode for process in children]
        launch_text=(output/"launch.log").read_text() if (output/"launch.log").exists() else ""
        result["clean_process_shutdown"] = (not forced and len(children)==2
            and all(process.returncode==0 for process in children)
            and launch_text.count("process has finished cleanly")>=4
            and "process has died" not in launch_text and "Traceback (most recent call last)" not in launch_text)
        if result["passed"] and (not result["cleanup_disarmed"] or not result["clean_process_shutdown"]):
            result["passed"]=False
            result["error"]="Software profile did not disarm/exit cleanly"
        trace.close();node.destroy_node();rclpy.shutdown()
        (output/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    return result

