"""Recording prerequisites are checked without preparing the ROS environment."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from ebva_recording.configuration import ConfigError, check_storage, load_preset, recorder_command
from ebva_recording.resource_publisher import METRIC_TOPICS


class RecordingConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.path = self.root / "preset.yaml"
        self.data = {"version": 2, "topics": ["/candidate_input"], "output_root": str(self.root / "bags")}

    def tearDown(self):
        self.temporary.cleanup()

    def load(self, **kwargs):
        self.path.write_text(json.dumps(self.data))
        return load_preset(self.path, **kwargs)

    def test_native_preset_has_no_overlay_dependency_or_writes(self):
        config = self.load()
        self.assertEqual(config["topics"], ["/candidate_input", *METRIC_TOPICS])
        self.assertFalse(config["output_root"].exists())
        self.assertNotIn("workspaces", config)
        self.data["ros_setup"] = "/not/used"
        with self.assertRaisesRegex(ConfigError, "Campos desconhecidos"):
            self.load()

    def test_old_schema_is_rejected(self):
        self.data['version'] = 1
        with self.assertRaisesRegex(ConfigError, 'version: 2'):
            self.load()

    def test_no_space_or_invalid_destination_are_rejected_without_writes(self):
        config = self.load()
        with patch("ebva_recording.configuration.available_bytes", return_value=0):
            with self.assertRaisesRegex(ConfigError, "Espaco livre"):
                check_storage(config)
        self.assertFalse(config["output_root"].exists())
        (self.root / "file").touch()
        config["output_root"] = self.root / "file/child"
        with self.assertRaisesRegex(ConfigError, "nao e diretorio"):
            check_storage(config)

    def test_duplicate_keys_and_bad_topics_are_rejected(self):
        self.path.write_text('version: 2\ntopics: [/a]\ntopics: [/b]\n')
        with self.assertRaisesRegex(ConfigError, "repetida"):
            load_preset(self.path)
        self.data["topics"] = ["--all"]
        with self.assertRaises(ConfigError):
            self.load()

    def test_duration_bounds_and_argv_preserve_spaces(self):
        for duration in (-1, True, float('inf')):
            with self.assertRaises(ConfigError):
                self.load(duration=duration)
        config = self.load(duration=0, output_root=self.root / "with spaces")
        args = recorder_command(config, config["output_root"] / "session")
        self.assertEqual(args[args.index("--output") + 1], str(self.root / "with spaces/session"))
        self.assertNotIn('--all', args)
        self.assertEqual(config["duration_s"], 0)
