"""Validação de sessão, argumentos e contrato numérico sem iniciar ROS."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ebva_recording.configuration import ConfigError, load_preset, recorder_command
from ebva_recording.resource_publisher import METRIC_TOPICS, TegrastatsReader, numeric_metrics


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.config = {"version": 2, "output_root": str(self.root/"bags"),
            "topics": ["/camera/image", "/lanes"]}

    def tearDown(self):
        self.temporary.cleanup()

    def load(self, **kwargs):
        path = self.root/"preset.yaml"
        path.write_text(json.dumps(self.config))  # JSON também é YAML válido.
        return load_preset(path, **kwargs)

    def test_load_does_not_create_output_and_always_records_metrics(self):
        data = self.load()
        self.assertFalse((self.root/"bags").exists())
        self.assertEqual(data["topics"], ["/camera/image", "/lanes", *METRIC_TOPICS])
        self.assertEqual(data["duration_s"], 60)

    def test_metrics_listed_explicitly_are_not_duplicated(self):
        self.config["topics"].append(METRIC_TOPICS[0])
        self.assertEqual(self.load()["topics"].count(METRIC_TOPICS[0]), 1)

    def test_unknown_keys_and_duplicate_topics_are_rejected(self):
        self.config["duraton_s"] = 10
        with self.assertRaises(ConfigError):
            self.load()
        del self.config["duraton_s"]
        self.config["topics"].append("/lanes")
        with self.assertRaises(ConfigError):
            self.load()

    def test_nonfinite_and_negative_duration_are_rejected(self):
        for value in (float("nan"), float("inf"), -1, True):
            with self.subTest(value=value), self.assertRaises(ConfigError):
                self.load(duration=value)

    def test_topic_and_prefix_cannot_inject_shell_arguments_or_paths(self):
        for topic in ("relative", "/image;rm", "--all", "/"):
            self.config["topics"] = [topic]
            with self.subTest(topic=topic), self.assertRaises(ConfigError):
                self.load()
        self.config["topics"] = ["/image"]
        self.config["session_prefix"] = "../outside"
        with self.assertRaises(ConfigError):
            self.load()

    def test_overrides_and_recorder_argv_preserve_path_with_spaces(self):
        data = self.load(duration=0, output_root=self.root/"with spaces")
        command = recorder_command(data, data["output_root"]/"session")
        self.assertEqual(command[command.index("--output")+1], str(self.root/"with spaces/session"))
        self.assertEqual(data["duration_s"], 0)
        self.assertNotIn("-a", command)
        self.assertTrue(all(topic in command for topic in METRIC_TOPICS))

    def test_recording_bounds_validate_cache_and_interval(self):
        for key, value in (("interval_s", 0), ("max_cache_size", -1), ("max_bag_size", 100), ("min_free_mib", 0)):
            original = copy.deepcopy(self.config)
            self.config[key] = value
            with self.subTest(key=key), self.assertRaises(ConfigError):
                self.load()
            self.config = original

    def test_numeric_metrics_preserve_units_and_unavailable_values(self):
        sample = {"cpu_percent": {"cpu": None}, "cpu_frequency_khz": {"cpu0": 729000},
            "memory_bytes": {"MemAvailable": 1024, "HugePages_Free": 0}, "network": {}, "disks": {}, "processes": [],
            "jetson": {"metrics": {"gpu_percent": 0, "temperatures_c": {"gpu": 50.5}, "power_mw": {"VDD_IN": {"current": 5000}}}}}
        values = {name: (value, unit) for name, value, unit in numeric_metrics(sample)}
        self.assertEqual(values["cpu/cpu"], (None, "percent"))
        self.assertEqual(values["memory/MemAvailable"], (1024, "bytes"))
        self.assertEqual(values["memory/HugePages_Free"], (0, "count"))
        self.assertEqual(values["jetson/gpu_percent"], (0, "percent"))
        self.assertEqual(values["power/VDD_IN/current"], (5000, "mW"))

    def test_monitor_without_tegrastats_is_supported(self):
        with patch("ebva_recording.resource_publisher.shutil.which", return_value=None):
            reader = TegrastatsReader(1)
        self.assertIsNone(reader.poll())
        self.assertIsNotNone(reader.error)
        reader.close()
        reader.close()


if __name__ == "__main__":
    unittest.main()
