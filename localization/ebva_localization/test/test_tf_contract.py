"""Declared topology/ports and independent observed TF identity/time checks."""
from copy import deepcopy
import math
import unittest

from ebva_localization.tf_contract import TfAudit, validate_profile
from tf_test_helpers import graph, profile, transform


class ProfileDeclarationTests(unittest.TestCase):
    def test_local_profile_does_not_require_map(self):
        original = profile()
        result = validate_profile(original)
        self.assertTrue(result["declaration_valid"])
        self.assertIsNone(result["global_frame"])
        self.assertNotIn("declaration_valid", original)

    def test_global_local_and_direct_global_are_alternatives(self):
        self.assertTrue(validate_profile(profile(True))["declaration_valid"])
        direct = profile(True)
        direct["local_frame"] = None
        direct["edges"] = [{"parent": "fixture_map", "child": direct["body_frame"],
            "kind": "dynamic", "publisher": "/direct_global", "max_age_sec": .25}]
        direct["state_ports"] = [direct["state_ports"][1]]
        direct["consumers"] = [direct["consumers"][1]]
        self.assertTrue(validate_profile(direct)["declaration_valid"])

    def test_message_only_profile_does_not_fabricate_tf(self):
        declaration = profile()
        declaration["require_tf"] = False
        declaration["edges"] = []
        result = validate_profile(declaration)
        self.assertEqual(result["tf_validation_scope"], "not_requested")

    def test_cannot_invent_map_or_unknown_state_frame(self):
        for field, value in (("role", "global"), ("frame", "camera"), ("body", "camera")):
            declaration = profile()
            declaration["state_ports"][0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_same_child_rejects_two_parents_or_authorities(self):
        for parent in ("fixture_odom", "fixture_map"):
            declaration = profile()
            extra = deepcopy(declaration["edges"][0])
            extra.update(parent=parent, publisher="/other_provider")
            declaration["edges"].append(extra)
            with self.subTest(parent=parent), self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_rejects_cycle_and_self_edge(self):
        for child in ("fixture_odom", "fixture_rear_axle_flu"):
            declaration = profile()
            declaration["edges"].append({"parent": "fixture_laser", "child": child,
                                          "publisher": "/cycle", "kind": "static"})
            with self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_requested_chains_must_exist(self):
        for global_frame in (False, True):
            declaration = profile(global_frame)
            declaration["edges"] = [e for e in declaration["edges"] if e["parent"] != "fixture_odom"]
            with self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_point_frame_body_and_type_must_match_consumer(self):
        for key, value in (("model_point", "camera_center"), ("frame", "fixture_map"),
                           ("body", "camera"), ("type", "geometry_msgs/PoseWithCovarianceStamped")):
            declaration = profile()
            declaration["consumers"][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_pose_cannot_offer_velocity_and_odom_twist_cannot_be_world_frame(self):
        declaration = profile(True)
        declaration["state_ports"][1]["provides"].append("longitudinal_velocity")
        with self.assertRaises(ValueError):
            validate_profile(declaration)
        declaration = profile()
        declaration["state_ports"][0]["twist_frame"] = "fixture_odom"
        with self.assertRaises(ValueError):
            validate_profile(declaration)

    def test_required_capabilities_and_age_policy_are_checked(self):
        for key, value in (("needs", ["yaw_rate"]), ("max_age_sec", .1)):
            declaration = profile()
            declaration["consumers"][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_profile(declaration)

    def test_republishing_time_and_unspecified_authority_lifetime_rejected(self):
        declaration = profile()
        declaration["state_ports"][0]["stamp"] = "reception"
        with self.assertRaises(ValueError):
            validate_profile(declaration)
        declaration = profile()
        declaration["static_lifetime"] = "forever"
        with self.assertRaises(ValueError):
            validate_profile(declaration)

    def test_bad_dynamic_time_and_missing_field_rejected(self):
        for timeout in (0., math.nan, math.inf, -.1):
            declaration = profile()
            declaration["edges"][0]["max_age_sec"] = timeout
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                validate_profile(declaration)
        declaration = profile()
        del declaration["state_ports"][0]["twist_frame"]
        with self.assertRaises(ValueError):
            validate_profile(declaration)


class TfObservationTests(unittest.TestCase):
    def setUp(self):
        self.declaration = profile()
        self.audit = TfAudit(self.declaration)
        self.endpoints = graph(self.declaration)
        self.audit.update_graph(self.endpoints)
        self.local_gid = "/tf_contract_fixture/local_estimator_gid"
        self.static_gid = "/tf_contract_fixture/extrinsics_gid"
        self.now = 10_000_000_000
        self.mono = 20_000_000_000

    def ingest(self, message=None, kind="dynamic", gid=None, now=None, mono=None):
        return self.audit.observe(message or transform(), kind, gid or self.local_gid,
                                  now or self.now, mono or self.mono)

    def all_valid(self):
        self.assertIsNone(self.ingest())
        self.assertIsNone(self.ingest(transform("fixture_rear_axle_flu", "fixture_laser", 0, .25),
                                      kind="static", gid=self.static_gid))

    def test_original_dynamic_and_zero_stamp_static_accepted_without_mutation(self):
        message = transform()
        before = deepcopy(message)
        self.assertIsNone(self.ingest(message))
        self.assertEqual(message, before)
        self.all_valid_after_dynamic()
        self.assertTrue(self.audit.report(self.now, self.mono)["observations_valid"])

    def all_valid_after_dynamic(self):
        self.assertIsNone(self.ingest(transform("fixture_rear_axle_flu", "fixture_laser", 0, .25),
                                      kind="static", gid=self.static_gid))

    def test_missing_discovery_is_not_judged_as_authenticated_input(self):
        self.audit.update_graph({})
        self.assertEqual(self.ingest(), "discovery_not_ready")
        self.assertEqual(self.audit.rejections, {})
        self.assertFalse(self.audit.report(self.now, self.mono)["discovery_ready"])

    def test_multiple_gids_for_same_fqn_are_ambiguous(self):
        self.endpoints["dynamic"]["/tf_contract_fixture/local_estimator"].add("another_gid")
        self.audit.update_graph(self.endpoints)
        report = self.audit.report(self.now, self.mono)
        self.assertFalse(report["discovery_ready"])
        self.assertIn("authority_ambiguous", [i["reason"] for i in report["issues"]])

    def test_wrong_publisher_kind_and_undeclared_edge_rejected(self):
        self.assertEqual(self.ingest(gid="rogue"), "unexpected_publisher_gid")
        self.assertEqual(self.ingest(kind="static", gid=self.static_gid), "tf_kind_mismatch")
        self.assertEqual(self.ingest(transform("other", "camera")), "undeclared_edge")

    def test_invalid_transform_and_quaternion_rejected_without_normalization(self):
        for invalid in (math.nan, math.inf):
            message = transform()
            message.transform.translation.x = invalid
            self.assertEqual(self.ingest(message), "nonfinite_transform")
        message = transform()
        message.transform.rotation.w = 2.
        self.assertEqual(self.ingest(message), "nonunit_quaternion")
        self.assertEqual(message.transform.rotation.w, 2.)

    def test_dynamic_expiry_future_and_invalid_stamp_rejected(self):
        self.assertEqual(self.ingest(transform(stamp_ns=self.now-250_000_000)), "transform_expired")
        self.assertEqual(self.ingest(transform(stamp_ns=self.now+20_000_001)), "transform_in_future")
        message = transform()
        message.header.stamp.nanosec = 1_000_000_000
        self.assertEqual(self.ingest(message), "invalid_stamp")

    def test_duplicates_and_reordering_do_not_renew_monotonic_deadline(self):
        self.assertIsNone(self.ingest())
        deadline = self.audit.observed[("fixture_odom", "fixture_rear_axle_flu")]["deadline_ns"]
        self.assertEqual(self.ingest(mono=self.mono+100_000_000), "transform_not_newer")
        self.assertEqual(self.ingest(transform(stamp_ns=self.now-1)), "transform_not_newer")
        self.assertEqual(self.audit.observed[("fixture_odom", "fixture_rear_axle_flu")]["deadline_ns"], deadline)

    def test_paused_ros_clock_still_expires_by_monotonic_time(self):
        self.all_valid()
        report = self.audit.report(self.now, self.mono+250_000_000)
        self.assertFalse(report["observations_valid"])
        self.assertIn("transform_expired", [i["reason"] for i in report["issues"]])

    def test_static_survives_time_but_not_endpoint_departure_or_replacement(self):
        self.all_valid()
        # Refresh the dynamic edge with a genuinely new acquisition.
        self.assertIsNone(self.ingest(transform(stamp_ns=self.now+5_000_000_000),
                                      now=self.now+5_000_000_000, mono=self.mono+5_000_000_000))
        self.assertTrue(self.audit.report(self.now+5_000_000_000, self.mono+5_000_000_000)["observations_valid"])
        self.endpoints["static"]["/tf_contract_fixture/extrinsics"] = {"replacement_gid"}
        self.audit.update_graph(self.endpoints)
        report = self.audit.report(self.now+5_000_000_000, self.mono+5_000_000_000)
        self.assertFalse(report["observations_valid"])
        self.assertIn("publishing_endpoint_changed_or_missing", [i["reason"] for i in report["issues"]])

    def test_clock_rewind_latches_until_recreated(self):
        self.all_valid()
        report = self.audit.report(self.now-1, self.mono+1)
        self.assertTrue(report["clock_reset_required"])
        self.assertEqual(self.ingest(now=self.now+1), "clock_reset_required")

    def test_queued_input_cannot_renew_after_discovery_delay(self):
        message = transform()
        self.assertEqual(self.audit.observe(message, "dynamic", self.local_gid,
            self.now, self.mono+300_000_000, received_at=(self.now, self.mono)),
            "transform_expired_before_discovery")

    def test_rejection_history_is_separate_from_endpoint_readiness(self):
        self.assertEqual(self.ingest(gid="rogue"), "unexpected_publisher_gid")
        self.all_valid()
        report = self.audit.report(self.now, self.mono)
        self.assertTrue(report["discovery_ready"])
        self.assertFalse(report["observations_valid"])
        self.assertEqual(report["rejections"]["unexpected_publisher_gid"], 1)
        self.assertFalse(report["authority_authentication"])
