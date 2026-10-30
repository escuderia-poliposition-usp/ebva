"""Explicit finite bench probe. ROS commands only; never opens serial devices.

Run only against the isolated suspended-car profile. Output must be new.
"""
import argparse
from contextlib import contextmanager
import os
import signal
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from ebva_control_msgs.msg import RcInput, ActuatorCommand, SelectedControl
from ebva_vehicle_control.nodes import qos


STEERING_STEPS = (.50, .45, .50, .55, .50, .40, .50, .60, .50)


def steering_step_plan(hold_s):
    """Bounded observation sequence, not a physical steering calibration."""
    if not 2. <= hold_s <= 5.:
        raise ValueError("steering-steps window must be 2..5 seconds per position")
    return [(value, hold_s) for value in STEERING_STEPS]


@contextmanager
def paused_control_process(name, proc_root=Path("/proc")):
    """Pause one allowed same-user producer; always resume via pidfd."""
    if name not in ("rc_acquisition", "command_authority"):
        raise ValueError("Only acquisition/authority may be paused by a bench probe")
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise RuntimeError("Silence probes require Linux pidfd support")
    matches = []
    for directory in proc_root.iterdir():
        if not directory.name.isdigit() or int(directory.name) == os.getpid():
            continue
        try:
            argv_bytes = (directory/"cmdline").read_bytes()
            argv = argv_bytes.decode(errors="replace").split("\0")
            if (directory.stat().st_uid == os.getuid()
                    and any(Path(arg).name == name for arg in argv[:2])):
                matches.append((int(directory.name), directory, argv_bytes))
        except OSError:
            continue
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one same-user "+name+" process")
    pid, directory, argv_bytes = matches[0]
    descriptor = os.pidfd_open(pid)
    try:
        if (directory/"cmdline").read_bytes() != argv_bytes:
            raise RuntimeError("Acquisition process changed before pause")
        try:
            signal.pidfd_send_signal(descriptor, signal.SIGSTOP)
            yield pid
        finally:
            try:
                signal.pidfd_send_signal(descriptor, signal.SIGCONT)
            except ProcessLookupError:
                pass  # Dead acquisition cannot recover; observer will reject it.
    finally:
        os.close(descriptor)


def paused_acquisition(proc_root=Path("/proc")):
    return paused_control_process("rc_acquisition", proc_root)


class Probe(Node):
    def __init__(self, output):
        from vesc_msgs.msg import VescStateStamped
        super().__init__("bench_observer")
        self.output = output
        self.trace = (output/"trace.jsonl").open("x")
        self.latest = {}
        self.events = []
        self.publish_auto = False
        self.armed = False
        self.erpm, self.servo = 0.0, 0.5
        self.pub = self.create_publisher(ActuatorCommand, "control/autonomy/actuators", qos())
        self.create_subscription(RcInput, "rc/input", self.rc, qos())
        self.create_subscription(SelectedControl, "control/selected", self.selected, qos())
        self.create_subscription(DiagnosticArray, "control/backend_status", self.backend_status, qos())
        self.create_subscription(Float64, "commands/motor/speed", lambda m: self.record("motor_command", {"erpm":m.data}), qos())
        self.create_subscription(Float64, "commands/servo/position", lambda m: self.record("servo_command", {"servo":m.data}), qos())
        self.create_subscription(Float64, "sensors/servo_position_command", lambda m: self.record("servo_driver_echo", {"servo":m.data}), 10)
        self.create_subscription(VescStateStamped, "sensors/core", self.telemetry, 10)
        self.timer = self.create_timer(.02, self.send)

    def record(self, kind, value):
        row = {"monotonic":time.monotonic(), "kind":kind, **value}
        self.latest[kind] = row
        self.events.append(row)
        self.trace.write(json.dumps(row)+"\n")
        return row

    def rc(self, message):
        self.record("rc", {"valid":message.valid, "sequence":message.sequence, "throttle":message.throttle_us,
                          "steering":message.steering_us, "rf_state":message.rf_state,
                          "reason":message.reason})

    def selected(self, message):
        self.record("selection", {"mode":message.mode, "reason":message.reason})

    def backend_status(self, message):
        for status in message.status:
            if status.name == "ebva_vesc_backend":
                values = {item.key:item.value for item in status.values}
                self.record("backend_status", {"enabled":values.get("enabled") == "True",
                                              "reason":status.message})

    def telemetry(self, message):
        state = message.state
        self.record("telemetry", {"speed_erpm":state.speed, "voltage":state.voltage_input,
                                  "current_motor":state.current_motor, "fault":state.fault_code})

    def send(self):
        if self.publish_auto:
            message = ActuatorCommand()
            message.header.stamp = self.get_clock().now().to_msg()
            message.erpm, message.servo = self.erpm, self.servo
            self.pub.publish(message)

    def spin_for(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.01)
            if self.armed:
                telemetry = self.latest.get("telemetry", {})
                if telemetry.get("fault", 0) != 0 or telemetry.get("voltage", 12.) < 10.5:
                    raise RuntimeError("VESC fault/voltage during bench; stopping")

    def call(self, name, value):
        client = self.create_client(SetBool, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError("Service unavailable: "+name)
            future = client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic()+3
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.01)
            if not future.done() or not future.result().success:
                raise RuntimeError("Service rejected: "+name+" "+str(future.result()))
        finally:
            self.destroy_client(client)

    def ready(self):
        self.spin_for(3)
        now = time.monotonic()
        rc, telemetry = self.latest.get("rc"), self.latest.get("telemetry")
        if not rc or not rc["valid"] or now-rc["monotonic"] > .2:
            raise RuntimeError("Fresh physical RC missing")
        if max(abs(rc["throttle"]-1500),abs(rc["steering"]-1500)) > 80:
            raise RuntimeError("RC must start neutral")
        if not telemetry or now-telemetry["monotonic"] > .5:
            raise RuntimeError("Fresh VESC telemetry missing")
        if telemetry["fault"] != 0 or telemetry["voltage"] < 10.5:
            raise RuntimeError("VESC fault or low 3S voltage: "+str(telemetry))
        for topic in ("commands/motor/speed","commands/servo/position","rc/input","control/selected"):
            if len(self.get_publishers_info_by_topic(topic)) != 1:
                raise RuntimeError("Publisher ownership mismatch: "+topic)

    def arm(self):
        self.publish_auto = True
        self.spin_for(.15)
        self.call("control/set_autonomy_enabled", True)
        self.spin_for(.1)
        self.call("control/set_actuation_enabled", True)
        self.armed = True
        self.spin_for(.15)

    def stop(self):
        self.armed = False
        self.erpm = 0.
        self.publish_auto = True
        self.spin_for(.15)
        try:
            self.call("control/set_actuation_enabled", False)
        finally:
            self.call("control/set_autonomy_enabled", False)
            self.publish_auto = False
            self.spin_for(.25)

    def steering_steps(self, hold_s):
        plan = steering_step_plan(hold_s)
        self.erpm, self.servo = 0., .5
        self.arm()
        points = []
        for index, (value, duration) in enumerate(plan, 1):
            self.servo = value
            start = time.monotonic()
            print(f"STEERING_STEP {index}/{len(plan)}: servo={value:.2f}, "
                  f"motor=0, hold={duration:g}s; observe wheels, keep RC neutral",
                  flush=True)
            while time.monotonic()-start < duration:
                self.spin_for(.05)
                if time.monotonic()-start > .3:
                    selected = self.latest.get("selection", {})
                    if selected.get("mode") != SelectedControl.AUTONOMY:
                        raise RuntimeError("Steering observation interrupted: autonomy not selected")
            settled = [e for e in self.events if e["monotonic"] >= start+.3]
            for kind, field, expected in (("motor_command", "erpm", 0.),
                                          ("servo_command", "servo", value),
                                          ("servo_driver_echo", "servo", value)):
                samples = [e[field] for e in settled if e["kind"] == kind]
                if not samples or any(abs(v-expected) > 1e-9 for v in samples):
                    raise RuntimeError("Missing/mismatched settled output: "+kind)
            points.append(self.record("steering_observation_point",
                                      {"step":index, "servo":value, "hold_s":duration}))
        return {"steering_steps":points, "motor_zero":True,
                "servo_center_assumed":False,
                "wheel_angles_require_operator_observation":True,
                "physical_symmetry_qualified":False}

    def rc_silence(self):
        self.erpm, self.servo = 0., .5
        self.arm()
        self.spin_for(.3)
        if self.latest.get("selection", {}).get("mode") != SelectedControl.AUTONOMY:
            raise RuntimeError("Autonomy not selected before RC silence")
        print("RC_SILENCE: motor=0, servo=.50; keep RC neutral; acquisition pauses for .7s",
              flush=True)
        with paused_acquisition() as pid:
            cutoff = time.monotonic()
            self.record("rc_acquisition_paused", {"pid":pid})
            self.spin_for(.7)
            loss = [e for e in self.events if e["kind"] == "selection"
                    and e["monotonic"] >= cutoff]
            stops = [e for e in loss if e["mode"] == SelectedControl.STOP
                     and e["reason"] == "rc_invalid_or_expired"]
            if not stops or stops[0]["monotonic"]-cutoff > .35:
                raise RuntimeError("RC silence did not select STOP within .35s")
            settled = [e for e in loss if e["monotonic"] >= cutoff+.35]
            if not settled or any(e["mode"] != SelectedControl.STOP for e in settled):
                raise RuntimeError("Selection did not remain stopped during RC silence")
            stop_delay = stops[0]["monotonic"]-cutoff
        resumed = time.monotonic()
        self.record("rc_acquisition_resumed", {"pid":pid})
        self.spin_for(.8)
        fresh = [e for e in self.events if e["kind"] == "rc" and e["valid"]
                 and e["monotonic"] >= resumed]
        if not fresh or self.latest.get("selection", {}).get("mode") != SelectedControl.AUTONOMY:
            raise RuntimeError("Fresh neutral RC did not restore autonomy")
        motors = [e["erpm"] for e in self.events
                  if e["kind"] == "motor_command" and e["monotonic"] >= cutoff]
        if not motors or any(value != 0. for value in motors):
            raise RuntimeError("Nonzero/missing motor command during RC silence test")
        return {"rc_silence_stop":True, "fresh_rc_recovery":True, "motor_zero":True,
                "pause_s":.7, "stop_observed_after_pause_s":stop_delay,
                "acceptance_s":.35, "rf_loss_tested":False,
                "physical_braking_tested":False}

    def authority_silence(self):
        self.erpm, self.servo = 0., .5
        self.arm()
        self.spin_for(.3)
        backend = self.latest.get("backend_status", {})
        if (self.latest.get("selection", {}).get("mode") != SelectedControl.AUTONOMY
                or not backend.get("enabled")
                or backend.get("reason") != "neutral_with_valid_autonomy"):
            raise RuntimeError("Autonomy/backend not active before authority silence")
        print("AUTHORITY_SILENCE: motor=0, servo=.50; keep RC neutral; authority pauses for .7s",
              flush=True)
        with paused_control_process("command_authority") as pid:
            cutoff = time.monotonic()
            self.record("command_authority_paused", {"pid":pid})
            self.spin_for(.7)
            if any(e["kind"] == "selection" and e["monotonic"] >= cutoff+.1 for e in self.events):
                raise RuntimeError("Selection publisher did not remain silent")
            live_rc = [e for e in self.events if e["kind"] == "rc" and e["valid"]
                       and e["monotonic"] >= cutoff+.4]
            if not live_rc:
                raise RuntimeError("Acquisition did not remain live during authority silence")
            states = [e for e in self.events if e["kind"] == "backend_status"
                      and e["monotonic"] >= cutoff]
            expired = [e for e in states if e["reason"] == "selection_expired" and e["enabled"]]
            if not expired or expired[0]["monotonic"]-cutoff > .4:
                raise RuntimeError("Independent backend expiry not observed within .4s")
            settled = [e for e in states if e["monotonic"] >= cutoff+.4]
            if not settled or any(e["reason"] != "selection_expired" or not e["enabled"]
                                  for e in settled):
                raise RuntimeError("Backend did not maintain independent expiry")
            delay = expired[0]["monotonic"]-cutoff
        resumed = time.monotonic()
        self.record("command_authority_resumed", {"pid":pid})
        self.spin_for(.8)
        selections = [e for e in self.events if e["kind"] == "selection"
                      and e["monotonic"] >= resumed and e["mode"] == SelectedControl.AUTONOMY]
        backend = self.latest.get("backend_status", {})
        if (not selections or not backend.get("enabled")
                or backend.get("reason") != "neutral_with_valid_autonomy"):
            raise RuntimeError("Fresh selection did not restore backend without rearming")
        motors = [e["erpm"] for e in self.events
                  if e["kind"] == "motor_command" and e["monotonic"] >= cutoff]
        if not motors or any(value != 0. for value in motors):
            raise RuntimeError("Nonzero/missing motor command during authority silence")
        return {"independent_backend_expiry":True, "rc_remained_live":True,
                "selection_silence_verified":True, "recovered_without_rearming":True,
                "motor_zero":True, "pause_s":.7,
                "backend_expiry_observed_after_pause_s":delay, "acceptance_s":.4,
                "firmware_watchdog_tested":False, "physical_braking_tested":False}

    def phase(self, name, window):
        self.ready()
        if name == "steering-steps":
            return self.steering_steps(window)
        if name == "rc-silence":
            return self.rc_silence()
        if name == "authority-silence":
            return self.authority_silence()
        if name == "baseline":
            return {"baseline":self.latest}
        if name == "pulse":
            self.arm()
            start = time.monotonic()
            self.erpm = -300.
            self.spin_for(2.)
            active = [e for e in self.events if e["kind"]=="motor_command" and e["monotonic"]>=start]
            if not any(e["erpm"] == -300. for e in active):
                raise RuntimeError("Autonomy pulse did not reach final backend")
            self.publish_auto = False
            cutoff = time.monotonic()
            self.spin_for(1.5)
            stopped = [e for e in self.events if e["kind"]=="motor_command" and e["monotonic"]>=cutoff+.3]
            if not stopped or any(e["erpm"] != 0. for e in stopped):
                raise RuntimeError("Autonomy expiry did not stop final motor command")
            speeds = [e["speed_erpm"] for e in self.events if e["kind"]=="telemetry" and e["monotonic"]>=start]
            return {"requested_erpm":-300., "autonomy_expiry_stop":True,
                    "observed_speed_min":min(speeds), "observed_speed_max":max(speeds),
                    "last_telemetry":self.latest.get("telemetry")}
        if name == "manual":
            self.servo = .53
            self.arm()
            print("READY_MANUAL_WINDOW: autonomy 0eRPM/servo.53; steer RC then return neutral", flush=True)
            start = time.monotonic()
            seen_manual, resumed = False, False
            while time.monotonic()-start < window:
                self.spin_for(.05)
                selected = self.latest.get("selection",{})
                if selected.get("mode") == SelectedControl.MANUAL:
                    seen_manual = True
                if seen_manual and selected.get("mode") == SelectedControl.AUTONOMY:
                    resumed = True
                    # Keep the whole advertised operator window open: ending at
                    # the first release can discard a subsequent left/right test.
            if not seen_manual or not resumed:
                raise RuntimeError("Physical RC override/neutral return not observed within window")
            final = self.latest.get("servo_command",{}).get("servo")
            if final != .53:
                raise RuntimeError("Neutral return did not restore autonomous steering")
            commands = [e["servo"] for e in self.events
                        if e["kind"]=="servo_command" and e["monotonic"]>=start]
            echoes = [e["servo"] for e in self.events
                      if e["kind"]=="servo_driver_echo" and e["monotonic"]>=start]
            if not echoes or max(echoes)-min(echoes) < .02:
                raise RuntimeError("Driver did not echo distinct steering commands")
            return {"physical_manual_override":seen_manual,"automatic_neutral_resume":resumed,
                    "final_autonomous_servo":final,
                    "operator_window_s":window,
                    "servo_command_range":[min(commands),max(commands)],
                    "servo_driver_echo_range":[min(echoes),max(echoes)],
                    "wheel_motion_requires_operator_observation":True}
        raise ValueError(name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("baseline","pulse","manual","steering-steps","rc-silence","authority-silence"), required=True)
    parser.add_argument("--window", type=float, default=45,
                        help="manual duration, or 2..5 seconds per steering-steps position")
    parser.add_argument("--output", type=Path, required=True)
    args=parser.parse_args()
    if not 1 <= args.window <= 60:
        parser.error("window must be 1..60 seconds")
    if args.phase == "steering-steps":
        try:
            steering_step_plan(args.window)
        except ValueError as exc:
            parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    probe=Probe(args.output)
    result={"phase":args.phase,"physical_hardware":True,"passed":False}
    try:
        result.update(probe.phase(args.phase,args.window))
        result["passed"]=True
    except Exception as exc:
        result["error"]=repr(exc)
    finally:
        try:
            probe.stop()
        except Exception as exc:
            result["stop_error"]=repr(exc)
            result["passed"]=False
        result["last"]=probe.latest
        (args.output/"result.json").write_text(json.dumps(result,indent=2)+"\n")
        probe.trace.close()
        probe.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
