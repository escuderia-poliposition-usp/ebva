"""Testes de contadores/deltas, PID reutilizado e encerramento da gravação."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from ebva_recording import resource_monitor as monitor

class ResourceMonitorTests(unittest.TestCase):
    def test_cpu_ignores_guest_double_count_and_waits_for_delta(self):
        before = monitor.parse_cpu("cpu 10 0 10 80 0 0 0 0 5 0\ncpu0 10 0 10 80 0 0 0 0\n")
        now = monitor.parse_cpu("cpu 30 0 20 150 0 0 0 0 10 0\ncpu0 30 0 20 150 0 0 0 0\n")
        self.assertEqual(before["cpu"]["total"], 100)
        self.assertIsNone(monitor.cpu_percent(before, {})["cpu"])
        self.assertAlmostEqual(monitor.cpu_percent(now, before)["cpu"], 30)

    def test_counter_reset_has_no_negative_rate(self):
        self.assertIsNone(monitor.rates({"bytes": 1}, {"bytes": 10}, 1, ["bytes"])["bytes_per_s"])
        self.assertEqual(monitor.rates({"bytes": 30}, {"bytes": 10}, 2, ["bytes"])["bytes_per_s"], 10)

    def test_memory_and_io_units(self):
        self.assertEqual(monitor.parse_key_values("VmRSS: 20 kB\nThreads: 4\nread_bytes: 13\n"),
                         {"VmRSS": 20480, "Threads": 4, "read_bytes": 13})

    def test_jetson_available_and_missing_metrics(self):
        result = monitor.parse_tegrastats("RAM 10/15656MB SWAP 0/7828MB GR3D_FREQ 42%@[306,306] gpu@53.25C VDD_IN 5428mW/5300mW")
        self.assertEqual(result["gpu_percent"], 42)
        self.assertIsNone(result["emc_percent"])
        self.assertEqual(result["temperatures_c"]["gpu"], 53.25)
        self.assertEqual(result["power_mw"]["VDD_IN"]["current"], 5428)
        self.assertIsNone(monitor.parse_tegrastats("unavailable")["gpu_percent"])

    @staticmethod
    def stat(ticks, start):
        fields = ["S"] + ["0"] * 21
        fields[11], fields[12], fields[19] = str(ticks), "0", str(start)
        return "42 (name with ) space) " + " ".join(fields)

    def test_process_stat_with_parentheses(self):
        self.assertEqual(monitor.parse_process_stat(self.stat(20, 100)), {"ticks": 20, "start_ticks": 100})

    def test_sampler_handles_pid_reuse_and_missing_optional_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            pid = root / "proc/42"
            pid.mkdir(parents=True)
            (pid/"cmdline").write_text("lane_detection_node\0--ros-args")
            (pid/"status").write_text("VmRSS: 20 kB\nThreads: 2\n")
            (root/"proc/stat").write_text("cpu 10 0 0 90\ncpu0 5 0 0 45\ncpu1 5 0 0 45\n")
            sampler = monitor.ResourceSampler(["lane_detection_node"], root/"proc", root/"sys")
            sampler.hz = 100
            with patch.object(monitor.time, "monotonic_ns", side_effect=[1000000000, 2000000000, 3000000000]):
                (pid/"stat").write_text(self.stat(100, 10))
                self.assertIsNone(sampler.sample()["processes"][0]["cpu_one_core_percent"])
                (pid/"stat").write_text(self.stat(200, 10))
                process = sampler.sample()["processes"][0]
                self.assertEqual(process["cpu_one_core_percent"], 100)
                self.assertEqual(process["cpu_machine_percent"], 50)
                self.assertEqual(process["rss_bytes"], 20480)
                self.assertIsNone(process["io_counters"])
                (pid/"stat").write_text(self.stat(1, 99))
                self.assertIsNone(sampler.sample()["processes"][0]["cpu_one_core_percent"])

    def test_disk_network_deltas_use_bytes_and_skip_partitions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/"proc/net").mkdir(parents=True)
            (root/"sys/block/nvme0n1").mkdir(parents=True)
            sampler = monitor.ResourceSampler([], root/"proc", root/"sys")
            with patch.object(monitor.time, "monotonic_ns", side_effect=[1000000000, 2000000000]):
                (root/"proc/net/dev").write_text("eth0: 100 1 0 0 0 0 0 0 200 2 0 0 0 0 0 0\n")
                (root/"proc/diskstats").write_text("259 0 nvme0n1 1 0 10 0 2 0 20 0 0 5 5\n259 1 nvme0n1p1 1 0 10 0 2 0 20 0 0 5 5\n")
                sampler.sample()
                (root/"proc/net/dev").write_text("eth0: 1100 2 0 0 0 0 0 0 2200 3 0 0 0 0 0 0\n")
                (root/"proc/diskstats").write_text("259 0 nvme0n1 2 0 12 0 3 0 24 0 0 7 7\n")
                result = sampler.sample()
                self.assertEqual(result["network"]["eth0"]["rates"]["rx_bytes_per_s"], 1000)
                self.assertEqual(result["disks"]["nvme0n1"]["rates"]["write_bytes_per_s"], 2048)
                self.assertNotIn("nvme0n1p1", result["disks"])

    def test_reserve_stops_writes_and_close_remains_idempotent(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(monitor.shutil, "which", return_value=None), patch.object(monitor.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout="MAXN", stderr="")):
            recorder = monitor.ResourceRecorder(directory, 1, 0, [])
            recorder.sample()
            self.assertEqual(recorder.count, 1)
            recorder.minimum_free = 1
            with patch.object(monitor.shutil, "disk_usage", return_value=SimpleNamespace(total=100, used=100, free=0)):
                recorder.sample()
            self.assertIsNotNone(recorder.error)
            recorder.sample()
            self.assertEqual(recorder.count, 1)
            recorder.close()
            recorder.close()
            summary = json.loads((recorder.directory/"summary.json").read_text())
            self.assertEqual(summary["samples"], 1)
            self.assertIsNotNone(summary["recording_error"])
            json.loads((recorder.directory/"resources.jsonl").read_text())


if __name__ == "__main__":
    unittest.main()
