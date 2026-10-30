"""ROS launch recording component, isolated from the vehicle's failure policy."""
import os
from pathlib import Path
import signal
import time

from .configuration import (ConfigError, check_storage, load_preset,
                            recorder_command, session_metadata)
from .resource_publisher import METRIC_TOPICS

_recording_actions = set()


def is_recording_action(action):
    return action in _recording_actions


def check_environment(config):
    """Static preflight; no DDS node, output directory or runtime lock is created."""
    from ament_index_python.packages import get_package_prefix
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    import shutil
    prefix = Path(get_package_prefix("ebva_recording")).resolve()
    if not (prefix / "lib/ebva_recording/resource_monitor").is_file():
        raise ConfigError("Compile ebva_recording para instalar resource_monitor")
    if config["output_root"].is_relative_to(prefix):
        raise ConfigError("Gravacoes precisam ficar fora da instalacao do pacote")
    if shutil.which("ros2") is None or "sqlite3" not in rosbag2_py.get_registered_writers():
        raise ConfigError("ros2 bag record / plugin sqlite3 indisponivel")
    for interface in ("statistics_msgs/msg/MetricsMessage", "std_msgs/msg/String"):
        get_message(interface)
    return {"available_bytes": check_storage(config), "package_prefix": str(prefix)}


class GraphProbe:
    """Short, nonblocking polls using an independent rclpy context."""

    def __init__(self, config):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        self.config = config
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node(f"recording_preflight_{os.getpid()}", context=self.context,
                         enable_rosout=False, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.required = set(config["topics"]) - set(METRIC_TOPICS)

    def poll(self, timeout=0):
        import rclpy
        from rosidl_runtime_py.utilities import get_message
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=timeout)
        occupied = [t for t in METRIC_TOPICS if self.node.count_publishers(t)]
        if occupied:
            raise ConfigError("Metricas ja possuem publicador: " + ", ".join(occupied))
        missing = sorted(t for t in self.required if not self.node.count_publishers(t))
        if not missing:
            types = dict(self.node.get_topic_names_and_types())
            for topic in self.required:
                if len(types.get(topic, [])) != 1:
                    raise ConfigError("Tipo ambiguo no topico " + topic)
                get_message(types[topic][0])
        return missing

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()


class RecordingSession:
    def __init__(self, config, standalone=False):
        self.config, self.standalone = config, standalone
        self.probe = None
        self.stream = None
        self.recorder = None
        self.monitor = None
        self.stopping = False
        self.started = False
        self.deadline = time.monotonic() + config["discovery_timeout_s"]

    def claim(self):
        import fcntl
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if not domain.isdecimal():
            raise ConfigError("ROS_DOMAIN_ID invalido")
        directory = Path.home() / ".cache/ebva_recording"
        directory.mkdir(parents=True, exist_ok=True)
        self.stream = (directory / f"recording_domain_{int(domain)}.lock").open("a+")
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.close()
            raise ConfigError("Ja existe uma gravacao neste dominio") from exc

    def close(self):
        if self.probe is not None:
            self.probe.close()
            self.probe = None
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def shutdown(self, context):
        self.stopping = True
        self.close()
        return []

    def stop(self, reason):
        from launch.actions import EmitEvent, LogInfo
        from launch.events import Shutdown
        from launch.events.process import SignalProcess
        if self.stopping:
            return []
        self.stopping = True
        if self.probe is not None:
            self.probe.close()
            self.probe = None
        actions = [LogInfo(msg="Gravacao encerrada: " + reason)]
        if self.started:
            actions.extend(EmitEvent(event=SignalProcess(signal_number=signal.SIGINT, process_matcher=lambda candidate, target=action: candidate is target))
                           for action in (self.recorder, self.monitor))
        else:
            self.close()
            if self.standalone:
                actions.append(EmitEvent(event=Shutdown(reason=reason)))
        return actions

    def process_exit(self, event, context):
        from launch.actions import EmitEvent
        from launch.events import Shutdown
        actions = self.stop(f"{event.process_name}: codigo {event.returncode}")
        if event.action is self.recorder:
            self.close()
            if self.standalone and not context.is_shutdown:
                actions.append(EmitEvent(event=Shutdown(reason="Sessao de gravacao finalizada")))
        return actions

    def poll(self, context):
        from launch.actions import OpaqueFunction, TimerAction
        from launch.logging import get_logger
        if context.is_shutdown or self.stopping:
            self.close()
            return []
        try:
            if self.probe is None:
                self.claim()
                self.probe = GraphProbe(self.config)
            missing = self.probe.poll()
            if missing:
                if time.monotonic() >= self.deadline:
                    raise ConfigError("Topicos sem publicador: " + ", ".join(missing))
                return [TimerAction(period=0.1, actions=[OpaqueFunction(function=self.poll)])]
            self.probe.close()
            self.probe = None
            return self.start()
        except Exception as exc:
            get_logger("ebva_recording").error(f"Falha na coleta; operacao preservada: {exc}")
            return self.stop(str(exc))

    def watch_storage(self, context):
        from launch.actions import OpaqueFunction, TimerAction
        if context.is_shutdown or self.stopping:
            return []
        try:
            check_storage(self.config)
        except (ConfigError, OSError) as exc:
            return self.stop(str(exc))
        return [TimerAction(period=1.0, actions=[OpaqueFunction(function=self.watch_storage)])]

    def start(self):
        from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
        from launch.event_handlers import OnProcessExit
        from launch_ros.actions import Node
        import json
        check_storage(self.config)
        self.config["output_root"].mkdir(parents=True, exist_ok=True)
        destination, metadata = session_metadata(self.config)
        if destination.exists():
            raise ConfigError("Destino da bag ja existe")
        self.recorder = ExecuteProcess(cmd=recorder_command(self.config, destination), output="screen",
                                       sigterm_timeout="15", sigkill_timeout="5")
        self.monitor = Node(package="ebva_recording", executable="resource_monitor",
                            name="recording_resource_monitor", output="screen", parameters=[{
                                "use_sim_time": False, "interval_s": self.config["interval_s"],
                                "process_patterns": self.config["process_patterns"],
                                "storage_path": str(self.config["output_root"]),
                                "session_metadata_json": json.dumps(metadata),
                            }])
        _recording_actions.update((self.recorder, self.monitor))
        self.started = True
        actions = [RegisterEventHandler(OnProcessExit(target_action=action, on_exit=self.process_exit))
                   for action in (self.recorder, self.monitor)]
        actions.extend([LogInfo(msg=f"Bag da sessao: {destination}"), self.recorder, self.monitor,
                        TimerAction(period=1.0, actions=[OpaqueFunction(function=self.watch_storage)])])
        if self.config["duration_s"]:
            actions.append(TimerAction(period=self.config["duration_s"], actions=[
                OpaqueFunction(function=lambda context: self.stop("Duracao atingida"))]))
        return actions


def launch_recording(context):
    from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
    from launch.event_handlers import OnShutdown
    from launch.logging import get_logger
    values = context.launch_configurations
    duration = values.get("duration_s", "")
    output = values.get("output_root", "")
    standalone = str(values.get("standalone", "true")).lower() == "true"
    try:
        config = load_preset(values["preset"], float(duration) if duration else None, output or None)
        check_environment(config)
    except Exception as exc:
        if standalone:
            raise
        # The integrated launch already checked static prerequisites before drivers.
        # A changed disk/dependency after that point is a collection-only failure.
        get_logger("ebva_recording").error(f"Falha na coleta; operacao preservada: {exc}")
        return []
    session = RecordingSession(config, standalone=standalone)
    # Let the launch loop start producers before the first readiness probe.
    return [RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=session.shutdown)])),
            TimerAction(period=0.1, actions=[OpaqueFunction(function=session.poll)])]
