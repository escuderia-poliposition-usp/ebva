"""Publica recursos ROS sem criar arquivos de métricas separados da bag."""
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time

from .resource_monitor import ResourceSampler, parse_tegrastats

METRIC_TOPICS = ["/system/resource_usage", "/system/resource_snapshot", "/system/resource_metadata"]
DEFAULT_PATTERNS = ["zed", "lane_detection_node", "rosbag2", "ros2 bag record", "resource_monitor", "vesc",
                    "ekf_node", "slam_toolbox", "autodrive2vehicle", "urg_node", "joy", "ackermann_mux"]


class TegrastatsReader:
    def __init__(self, interval_s):
        self.child, self.latest, self.buffer, self.error = None, None, b"", None
        executable = shutil.which("tegrastats")
        if executable is None:
            self.error = "tegrastats indisponivel"
            return
        try:
            self.child = subprocess.Popen([executable, "--interval", str(round(interval_s * 1000))],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            os.set_blocking(self.child.stdout.fileno(), False)
        except OSError as exc:
            self.close()
            self.error = str(exc)

    def poll(self):
        if self.child is None:
            return self.latest
        try:
            self.buffer += os.read(self.child.stdout.fileno(), 65536)
        except BlockingIOError:
            pass
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            text = raw.decode(errors="replace")
            self.latest = {"received_wall_time_ns": time.time_ns(),
                "received_monotonic_ns": time.monotonic_ns(), "raw": text,
                "metrics": parse_tegrastats(text)}
        if self.child.poll() is not None:
            self.error = "tegrastats terminou com codigo " + str(self.child.returncode)
        return self.latest

    def close(self):
        if self.child is not None:
            if self.child.poll() is None:
                self.child.terminate()
                try:
                    self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.child.kill()
                    self.child.wait(timeout=2)
            if self.child.stdout is not None:
                self.child.stdout.close()
            self.child = None


def numeric_metrics(sample):
    """Valores numéricos e unidades; o snapshot conserva os demais campos."""
    for name, value in sample["cpu_percent"].items():
        yield "cpu/" + name, value, "percent"
    for name, value in sample["cpu_frequency_khz"].items():
        yield "cpu_frequency/" + name, value, "kHz"
    for name, value in sample["memory_bytes"].items():
        yield "memory/" + name, value, "bytes" if name not in ("HugePages_Total", "HugePages_Free", "HugePages_Rsvd", "HugePages_Surp") else "count"
    for group in ("network", "disks"):
        for device, data in sample[group].items():
            for category, values in data.items():
                for name, value in values.items():
                    unit = "bytes" if "bytes" in name else "ms" if "time_ms" in name else "count"
                    if name.endswith("_per_s"):
                        unit += "/s"
                    yield "/".join((group, device, category, name)), value, unit
    for process in sample["processes"]:
        prefix = "process/" + str(process["pid"]) + "/" + str(process["start_ticks"])
        for name, unit in (("cpu_one_core_percent", "percent"), ("cpu_machine_percent", "percent"), ("rss_bytes", "bytes"), ("threads", "count")):
            yield prefix + "/" + name, process[name], unit
        for name, value in (process["io_counters"] or {}).items():
            yield prefix + "/io/" + name, value, "bytes" if name in ("rchar", "wchar", "read_bytes", "write_bytes", "cancelled_write_bytes") else "count"
    for name, value in sample.get("storage_bytes", {}).items():
        yield "storage/" + name, value, "bytes"
    jetson = sample.get("jetson")
    metrics = jetson["metrics"] if jetson else {}
    for name in ("gpu_percent", "emc_percent"):
        yield "jetson/" + name, metrics.get(name), "percent"
    for name, value in metrics.get("temperatures_c", {}).items():
        yield "temperature/" + name, value, "degC"
    for rail, values in metrics.get("power_mw", {}).items():
        for name, value in values.items():
            yield "power/" + rail + "/" + name, value, "mW"


def metadata(session_json, interval_s, patterns):
    report = {"schema_version": 1, "hostname": platform.node(), "platform": platform.platform(),
        "interval_s": interval_s, "process_patterns": patterns, "session": json.loads(session_json),
        "monitor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "sampler_sha256": hashlib.sha256(Path(__file__).with_name("resource_monitor.py").read_bytes()).hexdigest(),
        "environment": {k: os.environ.get(k) for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")},
        "conventions": {"cpu": "processo 100% por nucleo; machine dividido por nucleos online",
            "unavailable": "snapshot null; MetricsMessage average=NaN e sample_count=0",
            "statistics": "cada mensagem descreve uma amostra: average=value, sample_count=1",
            "clock": "window_start/stop usam tempo de parede; snapshot conserva monotonic_ns",
            "power": "VDD_IN e rail do modulo, nao consumo do carrinho inteiro"}}
    try:
        q = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, timeout=3)
        report["power_mode"] = {"returncode": q.returncode, "stdout": q.stdout, "stderr": q.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        report["power_mode"] = {"unavailable": str(exc)}
    return report


def main(args=None):
    import rclpy
    from builtin_interfaces.msg import Time
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from statistics_msgs.msg import MetricsMessage, StatisticDataPoint, StatisticDataType
    from std_msgs.msg import String

    def stamp(ns):
        return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)

    class Monitor(Node):
        def __init__(self):
            super().__init__("resource_monitor")
            self.interval = self.declare_parameter("interval_s", 1.0).value
            patterns = self.declare_parameter("process_patterns", DEFAULT_PATTERNS).value
            self.storage = Path(self.declare_parameter("storage_path", str(Path.home())).value).expanduser()
            session = self.declare_parameter("session_metadata_json", "{}").value
            if not math.isfinite(self.interval) or not 0.1 <= self.interval <= 3600:
                raise ValueError("interval_s deve ser 0.1..3600")
            self.sampler, self.reader = ResourceSampler(patterns), None
            self.previous_wall = None
            self.usage = self.create_publisher(MetricsMessage, METRIC_TOPICS[0], 1000)
            self.snapshot = self.create_publisher(String, METRIC_TOPICS[1], 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.meta = self.create_publisher(String, METRIC_TOPICS[2], qos)
            # Consultas que podem falhar precedem o início do processo filho.
            report = metadata(session, self.interval, patterns)
            self.reader = TegrastatsReader(self.interval)
            try:
                report["tegrastats_error"] = self.reader.error
                self.meta.publish(String(data=json.dumps(report, allow_nan=False)))
                self.timer = self.create_timer(self.interval, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            except BaseException:
                self.reader.close()
                raise
            self.get_logger().info("Publicando recursos ROS; nenhum arquivo de metricas separado")

        def tick(self):
            sample = self.sampler.sample()
            sample["jetson"] = self.reader.poll()
            sample["tegrastats_error"] = self.reader.error
            usage = shutil.disk_usage(self.storage)
            sample["storage_bytes"] = {"total": usage.total, "used": usage.used, "free": usage.free}
            self.snapshot.publish(String(data=json.dumps(sample, allow_nan=False)))
            stop = sample["wall_time_ns"]
            start = min(self.previous_wall, stop) if self.previous_wall is not None else stop
            for name, value, unit in numeric_metrics(sample):
                available = value is not None and math.isfinite(value)
                message = MetricsMessage(measurement_source_name=platform.node(), metrics_source=name, unit=unit,
                    window_start=stamp(start), window_stop=stamp(stop), statistics=[
                        StatisticDataPoint(data_type=StatisticDataType.STATISTICS_DATA_TYPE_AVERAGE, data=float(value) if available else math.nan),
                        StatisticDataPoint(data_type=StatisticDataType.STATISTICS_DATA_TYPE_SAMPLE_COUNT, data=1.0 if available else 0.0)])
                self.usage.publish(message)
            self.previous_wall = stop

        def close(self):
            if self.reader:
                self.reader.close()

    rclpy.init(args=args)
    node = None
    try:
        node = Monitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
