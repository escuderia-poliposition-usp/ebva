"""Coleta local de recursos Linux/Jetson; ROS fica somente no adaptador main."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import time
import uuid


def read_text(path):
    try:
        return Path(path).read_text(errors="replace")
    except OSError:
        return ""


def parse_cpu(text):
    result = {}
    for line in text.splitlines():
        fields = line.split()
        if fields and re.fullmatch(r"cpu\d*", fields[0]):
            ticks = [int(value) for value in fields[1:9]]
            result[fields[0]] = {"total": sum(ticks), "idle": sum(ticks[3:5])}
    return result


def cpu_percent(current, previous):
    result = {}
    for name, now in current.items():
        before = previous.get(name)
        total = now["total"] - before["total"] if before else 0
        idle = now["idle"] - before["idle"] if before else 0
        result[name] = 100.0 * (total - idle) / total if total > 0 and 0 <= idle <= total else None
    return result


def parse_key_values(text):
    result = {}
    for line in text.splitlines():
        key, separator, value = line.partition(":")
        fields = value.split()
        if separator and fields and fields[0].isdigit():
            result[key] = int(fields[0]) * (1024 if fields[-1] == "kB" else 1)
    return result


def rates(current, previous, seconds, keys):
    return {key + "_per_s": (current[key] - previous[key]) / seconds
            if previous and key in previous and seconds > 0 and current[key] >= previous[key]
            else None for key in keys}


def parse_tegrastats(line):
    result = {"temperatures_c": {}, "power_mw": {}}
    for label in ("RAM", "SWAP"):
        match = re.search(r"\b" + label + r" (\d+)/(\d+)MB", line)
        if match:
            result[label.lower() + "_used_mb"] = int(match[1])
            result[label.lower() + "_total_mb"] = int(match[2])
    for label, key in (("GR3D_FREQ", "gpu_percent"), ("EMC_FREQ", "emc_percent")):
        match = re.search(r"\b" + label + r" (\d+)%", line)
        result[key] = int(match[1]) if match else None
    for name, value in re.findall(r"\b([\w]+)@([\d.]+)C", line):
        result["temperatures_c"][name] = float(value)
    for name, current, average in re.findall(r"\b(VDD_\w+) (\d+)mW/(\d+)mW", line):
        result["power_mw"][name] = {"current": int(current), "average": int(average)}
    return result


def parse_process_stat(text):
    # comm pode conter espaços e parênteses. Os campos seguintes começam no 3.
    tail = text[text.rindex(")") + 2:].split()
    return {"ticks": int(tail[11]) + int(tail[12]), "start_ticks": int(tail[19])}


class ResourceSampler:
    def __init__(self, patterns, proc=Path("/proc"), sys=Path("/sys")):
        self.proc, self.sys = Path(proc), Path(sys)
        self.patterns = tuple(patterns)
        self.hz = os.sysconf("SC_CLK_TCK")
        self.previous_cpu, self.previous_processes = {}, {}
        self.previous_net, self.previous_disk = {}, {}
        self.previous_time = None

    def sample(self):
        now = time.monotonic_ns()
        seconds = (now - self.previous_time) / 1e9 if self.previous_time is not None else 0
        cpu = parse_cpu(read_text(self.proc / "stat"))
        memory = parse_key_values(read_text(self.proc / "meminfo"))
        result = {"wall_time_ns": time.time_ns(), "monotonic_ns": now,
                  "interval_s": seconds or None, "cpu_percent": cpu_percent(cpu, self.previous_cpu),
                  "memory_bytes": memory, "processes": [], "network": {}, "disks": {},
                  "cpu_frequency_khz": {}, "pressure": {}}
        for path in (self.sys / "devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
            value = read_text(path).strip()
            if value.isdigit():
                result["cpu_frequency_khz"][path.parents[1].name] = int(value)
        for kind in ("cpu", "memory", "io"):
            result["pressure"][kind] = read_text(self.proc / "pressure" / kind).strip() or None
        net = {}
        for line in read_text(self.proc / "net/dev").splitlines():
            name, separator, values = line.partition(":")
            if separator:
                values = [int(v) for v in values.split()]
                net[name.strip()] = dict(zip(("rx_bytes", "rx_packets", "rx_errors", "rx_dropped",
                                               "tx_bytes", "tx_packets", "tx_errors", "tx_dropped"),
                                              values[:4] + values[8:12]))
        for name, counters in net.items():
            result["network"][name] = {"counters": counters,
                "rates": rates(counters, self.previous_net.get(name), seconds, counters)}
        disk = {}
        for line in read_text(self.proc / "diskstats").splitlines():
            fields = line.split()
            if len(fields) >= 14 and (self.sys / "block" / fields[2]).exists() and not fields[2].startswith(("loop", "ram", "zram")):
                disk[fields[2]] = {"read_bytes": int(fields[5]) * 512,
                    "write_bytes": int(fields[9]) * 512, "read_operations": int(fields[3]),
                    "write_operations": int(fields[7]), "io_time_ms": int(fields[12]),
                    "in_flight": int(fields[11])}
        for name, counters in disk.items():
            result["disks"][name] = {"counters": counters,
                "rates": rates(counters, self.previous_disk.get(name), seconds,
                               ("read_bytes", "write_bytes", "read_operations", "write_operations", "io_time_ms"))}
        processes = {}
        core_count = max(1, len(cpu) - 1)
        for path in self.proc.glob("[0-9]*"):
            command = read_text(path / "cmdline").replace("\0", " ").strip()
            if not command or not any(pattern in command for pattern in self.patterns):
                continue
            try:
                stat = parse_process_stat(read_text(path / "stat"))
                before = self.previous_processes.get(path.name)
                valid = before and before["start_ticks"] == stat["start_ticks"] and seconds > 0 and stat["ticks"] >= before["ticks"]
                one_core = (stat["ticks"] - before["ticks"]) / self.hz / seconds * 100 if valid else None
                status = parse_key_values(read_text(path / "status"))
                processes[path.name] = stat
                result["processes"].append({"pid": int(path.name), "command": command,
                    "start_ticks": stat["start_ticks"], "cpu_one_core_percent": one_core,
                    "cpu_machine_percent": one_core / core_count if one_core is not None else None,
                    "rss_bytes": status.get("VmRSS"), "threads": status.get("Threads"),
                    "io_counters": parse_key_values(read_text(path / "io")) or None})
            except (OSError, ValueError, IndexError):
                continue  # Processo pode terminar durante a leitura.
        self.previous_cpu, self.previous_processes = cpu, processes
        self.previous_net, self.previous_disk, self.previous_time = net, disk, now
        return result


class ResourceRecorder:
    def __init__(self, output_root, interval_s, min_free_mib, patterns, config_files=()):
        self.root = Path(output_root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.minimum_free = int(min_free_mib * 1024**2)
        if shutil.disk_usage(self.root).free < self.minimum_free:
            raise RuntimeError("Espaco livre abaixo da reserva configurada para a coleta")
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.directory = self.root / (stamp + "_" + uuid.uuid4().hex[:8])
        self.directory.mkdir()
        self.samples = (self.directory / "resources.jsonl").open("x", buffering=1)
        self.tegrastats = (self.directory / "tegrastats.jsonl").open("x", buffering=1)
        self.sampler = ResourceSampler(patterns)
        self.child, self.buffer, self.latest_jetson = None, b"", None
        self.count, self.error, self.closed = 0, None, False
        self.started = time.monotonic_ns()
        self.metadata = {"schema_version": 1, "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "hostname": platform.node(), "platform": platform.platform(), "pid": os.getpid(),
            "interval_s": interval_s, "min_free_mib": min_free_mib, "process_patterns": list(patterns),
            "cpu_convention": "sistema 0..100 por nucleo; processo 100 por nucleo, machine dividido por nucleos online",
            "power_convention": "rails do modulo; VDD_IN nao representa o carrinho inteiro",
            "unavailable": "null ou campo ausente; primeira amostra nao tem deltas",
            "environment": {key: os.environ.get(key) for key in ("ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")},
            "files": {}, "tegrastats_executable": shutil.which("tegrastats")}
        for name in [__file__, *config_files]:
            path = Path(name).expanduser()
            try:
                self.metadata["files"][str(path)] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
            except OSError as exc:
                self.metadata["files"][str(path)] = {"unavailable": str(exc)}
        try:
            power = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, timeout=3)
            self.metadata["power_mode"] = {"returncode": power.returncode, "stdout": power.stdout, "stderr": power.stderr}
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.metadata["power_mode"] = {"unavailable": str(exc)}
        executable = self.metadata["tegrastats_executable"]
        if executable:
            try:
                self.child = subprocess.Popen([executable, "--interval", str(max(100, round(interval_s * 1000)))],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                os.set_blocking(self.child.stdout.fileno(), False)
            except OSError as exc:
                self.metadata["tegrastats_error"] = str(exc)
        try:
            (self.directory / "metadata.json").write_text(json.dumps(self.metadata, indent=2) + "\n")
        except OSError:
            self.close()
            raise

    def drain_tegrastats(self):
        if self.child is None or self.child.stdout is None:
            return
        try:
            chunk = os.read(self.child.stdout.fileno(), 65536)
        except BlockingIOError:
            return
        self.buffer += chunk
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            raw = line.decode(errors="replace")
            self.latest_jetson = {"received_wall_time_ns": time.time_ns(),
                "received_monotonic_ns": time.monotonic_ns(), "raw": raw,
                "metrics": parse_tegrastats(raw)}
            self.tegrastats.write(json.dumps(self.latest_jetson) + "\n")

    def sample(self):
        if self.error or self.closed:
            return
        try:
            usage = shutil.disk_usage(self.directory)
            if usage.free < self.minimum_free:
                raise RuntimeError("Reserva de disco atingida; coleta interrompida, no permanece ativo")
            self.drain_tegrastats()
            data = self.sampler.sample()
            data["storage_bytes"] = {"total": usage.total, "used": usage.used, "free": usage.free}
            data["jetson"] = self.latest_jetson
            data["tegrastats_returncode"] = self.child.poll() if self.child else None
            self.samples.write(json.dumps(data, allow_nan=False) + "\n")
            self.count += 1
        except (OSError, RuntimeError, ValueError) as exc:
            self.error = str(exc)
            self.stop_child()

    def stop_child(self):
        if self.child is not None:
            if self.child.poll() is None:
                self.child.terminate()
                try:
                    self.child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.child.kill()
                    self.child.wait(timeout=2)
            if self.child.stdout is not None and not self.child.stdout.closed:
                self.child.stdout.close()
            self.child = None

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.drain_tegrastats()
        except (OSError, ValueError):
            pass
        self.stop_child()
        for stream in (self.samples, self.tegrastats):
            try:
                stream.close()
            except OSError:
                pass
        summary = {"ended_at_utc": datetime.now(timezone.utc).isoformat(), "samples": self.count,
                   "elapsed_s": (time.monotonic_ns() - self.started) / 1e9, "recording_error": self.error}
        try:
            (self.directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        except OSError:
            pass


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class Monitor(Node):
        def __init__(self):
            super().__init__("resource_monitor")
            defaults = {"output_root": "~/performance_logs", "interval_s": 1.0,
                        "min_free_mib": 1024, "duration_s": 0.0,
                        "process_patterns": ["zed", "lane_detection_node", "rosbag2", "resource_monitor", "vesc", "ekf_node", "slam_toolbox", "autodrive2vehicle", "urg_node"],
                        "config_files": [str(Path(__file__).resolve())]}
            values = {name: self.declare_parameter(name, default).value for name, default in defaults.items()}
            if not 0.1 <= values["interval_s"] <= 3600 or values["duration_s"] < 0 or values["min_free_mib"] < 0:
                raise ValueError("interval_s deve ser 0.1..3600, duration_s e min_free_mib nao negativos")
            self.recorder = ResourceRecorder(values["output_root"], values["interval_s"],
                values["min_free_mib"], values["process_patterns"], values["config_files"])
            self.duration = values["duration_s"]
            self.done, self.warned = False, False
            self.get_logger().info("Recursos em " + str(self.recorder.directory))
            self.timer = self.create_timer(values["interval_s"], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.tick()

        def tick(self):
            self.recorder.sample()
            if self.recorder.error and not self.warned:
                self.get_logger().error("Coleta parou: " + self.recorder.error)
                self.warned = True
            if self.duration and (time.monotonic_ns() - self.recorder.started) / 1e9 >= self.duration:
                self.done = True

    rclpy.init(args=args)
    node = None
    try:
        node = Monitor()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.recorder.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
