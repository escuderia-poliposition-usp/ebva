"""Conservative source inference on Humble: never ignore another endpoint."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from tf2_msgs.msg import TFMessage
from ebva_localization.state_tf_audit import StateTfAudit
from ebva_localization.tf_contract import TfAudit
from tf_test_helpers import graph, profile, transform


def observer():
    fake = SimpleNamespace(audit=TfAudit(profile()), pending=deque(maxlen=64),
        refresh_graph=lambda: None, publish_status=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    fake.stream_gid = lambda kind: StateTfAudit.stream_gid(fake, kind)
    fake.stream_issues = lambda: StateTfAudit.stream_issues(fake)
    return fake


class ConservativeInferenceTests(unittest.TestCase):
    def test_different_fqn_on_same_stream_prevents_any_attribution(self):
        fake = observer()
        endpoints = graph(profile())
        endpoints["dynamic"]["/unselected/rogue"] = {"rogue_gid"}
        fake.audit.update_graph(endpoints)
        with patch("ebva_localization.state_tf_audit.time.monotonic_ns", return_value=20_000_000_000):
            StateTfAudit.on_tf(fake, TFMessage(transforms=[transform()]), "dynamic")
        self.assertEqual(fake.audit.observed, {})
        self.assertEqual(fake.audit.rejections, {"ambiguous_stream": 1})

    def test_pending_static_from_replaced_endpoint_is_never_reassigned(self):
        fake = observer()
        endpoints = graph(profile())
        endpoints["dynamic"] = {}
        fake.audit.update_graph(endpoints)
        message = transform("fixture_rear_axle_flu", "fixture_laser", 0, .25)
        with patch("ebva_localization.state_tf_audit.time.monotonic_ns", return_value=20_000_000_000):
            StateTfAudit.on_tf(fake, TFMessage(transforms=[message]), "static")
        self.assertEqual(len(fake.pending), 1)
        endpoints = graph(profile())
        endpoints["static"]["/tf_contract_fixture/extrinsics"] = {"replacement_gid"}
        fake.audit.update_graph(endpoints)
        StateTfAudit.tick(fake)
        self.assertEqual(fake.audit.observed, {})
        self.assertEqual(fake.audit.rejections, {"pending_endpoint_unknown_or_changed": 1})

    def test_sample_received_before_source_is_known_is_not_qualified_later(self):
        fake = observer()
        fake.audit.update_graph({})
        message = transform("fixture_rear_axle_flu", "fixture_laser", 0, .25)
        with patch("ebva_localization.state_tf_audit.time.monotonic_ns", return_value=20_000_000_000):
            StateTfAudit.on_tf(fake, TFMessage(transforms=[message]), "static")
        self.assertEqual(len(fake.pending), 1)
        fake.audit.update_graph(graph(profile()))
        StateTfAudit.tick(fake)
        self.assertEqual(fake.audit.observed, {})
        self.assertEqual(fake.audit.rejections, {"pending_endpoint_unknown_or_changed": 1})
