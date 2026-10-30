"""Directed SI-boundary, numerical equivalence and failure-recovery checks."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from sensor_msgs.msg import Imu

from ebva_localization.imu_odometry import ImuOdometry
from ebva_localization.imu_prior import ImuPrior, ImuPriorPolicy, ImuSample

ROOT = Path(__file__).resolve().parents[1]


def fixture_values():
    return json.loads((ROOT/"config/imu_prior/si_fixture.json").read_text())


def policy(**changes):
    return ImuPriorPolicy(**{**fixture_values(), **changes})


def sample(stamp=10_000_000_000, *, a=(0., 0., 9.806), yaw=0., gyro=(0., 0., 0.), **changes):
    return ImuSample(stamp, "fixture_base", (0., 0., math.sin(yaw/2), math.cos(yaw/2)),
                     tuple(a), tuple(gyro), **changes)


def message(stamp=10_000_000_000, **kwargs):
    source = sample(stamp, **kwargs)
    msg = Imu()
    msg.header.frame_id = source.frame
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = source.quaternion
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = source.acceleration
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = source.angular_velocity
    return msg


def calibrated(**changes):
    prior = ImuPrior(policy(**changes))
    for i in range(4):
        result, reason = prior.process(sample(10_000_000_000+i*50_000_000), 10_000_000_000+i*50_000_000)
        assert result is None
    assert reason == "calibration_complete"
    return prior


def fake_node():
    fake = SimpleNamespace(policy=policy(), prior=ImuPrior(policy()),
                           publisher=Mock(), status_publisher=Mock(),
                           accepted_count=0, rejected_count=0, now_ns=10_000_000_000)
    fake.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=fake.now_ns))
    return fake


class ImuPriorPolicyTests(unittest.TestCase):
    def test_startup_relative_parent_cannot_relabel_the_orientation_world(self):
        with self.assertRaises(ValueError):
            policy(parent_frame="fixture_orientation_world")

    def test_only_explicit_si_specific_force_and_stationarity_are_supported(self):
        cases = ({"input_acceleration_units": "g"}, {"input_angular_velocity_units": "deg/s"},
                 {"acceleration_content": "gravity_removed"},
                 {"calibration_stationary_asserted": False},
                 {"input_frame": "camera"}, {"child_frame": "camera"},
                 {"parent_frame": "fixture_base"}, {"orientation_reference_frame": ""},
                 {"orientation_reference_frame": "fixture_base"},
                 {"calib_samples": 1}, {"calib_samples": True},
                 {"covariance_provenance": "calibrated"}, {"zupt_enable": 1},
                 {"future_tolerance_sec": .2}, {"future_tolerance_sec": .3})
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                policy(**changes)

    def test_nonfinite_invalid_policy_and_nanosecond_resolution_are_rejected(self):
        names = ("max_age_sec", "max_integration_dt_sec", "future_tolerance_sec", "max_speed",
                 "calibration_accel_tolerance_mps2", "calibration_gyro_threshold_rps",
                 "calibration_orientation_tolerance_rad", "expected_specific_force_norm_mps2",
                 "specific_force_norm_tolerance_mps2", "zupt_accel_thresh", "zupt_gyro_thresh")
        for name in names:
            for value in (math.nan, math.inf, -1.):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    policy(**{name: value})
        for name in ("max_age_sec", "max_integration_dt_sec"):
            with self.assertRaises(ValueError):
                policy(**{name: 1e-12})

    def test_explicit_covariance_must_be_structurally_valid_and_nonzero(self):
        for name in ("pose_covariance", "twist_covariance"):
            matrices = [[0.]*36, [math.nan]*36, [1.]*35]
            asymmetric = np.eye(6); asymmetric[0, 1] = .1
            indefinite = np.eye(6); indefinite[0, 1] = indefinite[1, 0] = 2.
            matrices += [asymmetric.ravel().tolist(), indefinite.ravel().tolist()]
            for matrix in matrices:
                with self.subTest(name=name), self.assertRaises(ValueError):
                    policy(**{name: matrix})
        self.assertEqual(policy(covariance_provenance="research_assumption").covariance_provenance,
                         "research_assumption")


class ImuPriorNumericsTests(unittest.TestCase):
    def test_translational_regression_reference(self):
        record = json.loads((ROOT/"test/data/imu-prior-regression.json").read_text())
        prior = ImuPrior(policy())
        compared = 0
        for index, pair in enumerate(record["pairs"]):
            value, reason = prior.process(sample(pair["stamp_ns"], a=pair["a"], yaw=pair["yaw"],
                                                 gyro=(0., 0., pair["gz"])), pair["stamp_ns"])
            if index < 4:
                self.assertIsNone(value)
            else:
                self.assertEqual(reason, "measurement_integrated")
                np.testing.assert_allclose(value["position"], pair["position"], rtol=1e-12, atol=1e-12)
                np.testing.assert_allclose(value["body_velocity"], pair["body_velocity"], rtol=1e-12, atol=1e-12)
                self.assertAlmostEqual(value["yaw"], pair["yaw"], places=12)
                compared += 1
        self.assertEqual(compared, 22)
        self.assertLessEqual(np.linalg.norm(prior.vel), 4.000000000001)

    def test_no_odometry_exists_during_or_at_end_of_calibration(self):
        prior = ImuPrior(policy())
        for i in range(4):
            value, reason = prior.process(sample(10_000_000_000+i*50_000_000), 10_000_000_000+i*50_000_000)
            self.assertIsNone(value)
            self.assertEqual(reason, "calibration_complete" if i == 3 else "calibrating")
        value, _ = prior.process(sample(10_200_000_000), 10_200_000_000)
        self.assertEqual(value["position"], (0., 0.))

    def test_zupt_and_speed_cap_remain_explicit_method_choices(self):
        a, b = calibrated(zupt_enable=True), calibrated(zupt_enable=False)
        for prior in (a, b):
            prior.process(sample(10_200_000_000, a=(1., 0., 9.806)), 10_200_000_000)
            prior.process(sample(10_250_000_000), 10_250_000_000)
        np.testing.assert_array_equal(a.vel, [0., 0.])
        self.assertGreater(b.vel[0], 0.)
        capped = calibrated(max_speed=.1)
        capped.process(sample(10_200_000_000, a=(100., 0., 9.806)), 10_200_000_000)
        self.assertAlmostEqual(np.linalg.norm(capped.vel), .1)

    def test_heading_is_startup_relative_and_twist_rotates_to_body(self):
        prior = ImuPrior(policy())
        for i in range(4):
            prior.process(sample(10_000_000_000+i*50_000_000, yaw=math.pi/2), 10_000_000_000+i*50_000_000)
        value, _ = prior.process(sample(10_200_000_000, a=(1., 0., 9.806), yaw=math.pi/2), 10_200_000_000)
        self.assertAlmostEqual(value["yaw"], 0.)
        self.assertAlmostEqual(value["position"][0], .0025)
        self.assertAlmostEqual(value["position"][1], 0., places=12)


class ImuPriorCalibrationTests(unittest.TestCase):
    def test_all_axis_gyro_motion_resets_startup_accumulation(self):
        for axis in range(3):
            prior = ImuPrior(policy())
            prior.process(sample(), 10_000_000_000)
            gyro = [0., 0., 0.]; gyro[axis] = .05
            value, reason = prior.process(sample(10_050_000_000, gyro=gyro), 10_050_000_000)
            self.assertIsNone(value)
            self.assertEqual(reason, "calibration_angular_motion")
            self.assertEqual(prior.calib_count, 0)

    def test_force_or_orientation_changes_reset_accumulation(self):
        for changed in (sample(10_050_000_000, a=(.3, 0., 9.806)), sample(10_050_000_000, yaw=.05)):
            prior = ImuPrior(policy()); prior.process(sample(), 10_000_000_000)
            _, reason = prior.process(changed, changed.stamp_ns)
            self.assertEqual(reason, "calibration_unstable")
            self.assertEqual(prior.calib_count, 0)

    def test_gravity_removed_acceleration_fails_declared_specific_force_contract(self):
        prior = ImuPrior(policy())
        _, reason = prior.process(sample(a=(0., 0., 0.)), 10_000_000_000)
        self.assertEqual(reason, "calibration_specific_force_mismatch")
        self.assertFalse(prior.calibrated)

    def test_startup_gap_restarts_contiguous_calibration(self):
        prior = ImuPrior(policy())
        prior.process(sample(), 10_000_000_000)
        prior.process(sample(10_300_000_000), 10_300_000_000)
        self.assertEqual(prior.calib_count, 1)
        self.assertFalse(prior.calibrated)

    def test_q_and_minus_q_do_not_look_like_orientation_motion(self):
        prior = ImuPrior(policy())
        prior.process(sample(), 10_000_000_000)
        current = replace(sample(10_050_000_000), quaternion=(0., 0., 0., -1.))
        self.assertEqual(prior.process(current, current.stamp_ns)[1], "calibrating")
        self.assertEqual(prior.calib_count, 2)

    def test_invalid_calibration_sample_does_not_consume_stamp(self):
        prior = ImuPrior(policy())
        bad = replace(sample(), quaternion=(0., 0., 0., 0.))
        self.assertEqual(prior.process(bad, bad.stamp_ns)[1], "nonunit_quaternion")
        self.assertEqual(prior.process(sample(), 10_000_000_000)[1], "calibrating")


class ImuPriorValidityTests(unittest.TestCase):
    def test_zero_scaled_nan_inf_quaternion_and_imu_cannot_poison_state(self):
        cases = [
            (replace(sample(), quaternion=(0., 0., 0., 0.)), "nonunit_quaternion"),
            (replace(sample(), quaternion=(0., 0., 0., 2.)), "nonunit_quaternion"),
            (replace(sample(), quaternion=(0., 0., math.nan, 1.)), "nonfinite_imu"),
            (replace(sample(), acceleration=(math.nan, 0., 9.806)), "nonfinite_imu"),
            (replace(sample(), acceleration=(0., math.inf, 9.806)), "nonfinite_imu"),
            (replace(sample(), angular_velocity=(0., 0., math.nan)), "nonfinite_imu"),
        ]
        for bad, expected in cases:
            prior = ImuPrior(policy())
            self.assertEqual(prior.process(bad, 10_000_000_000)[1], expected)
            np.testing.assert_array_equal(prior.pos, [0., 0.])
            self.assertFalse(prior.calibrated)

    def test_unavailable_components_reject_instead_of_assuming_identity_or_zero(self):
        for field in ("orientation_available", "acceleration_available", "angular_velocity_available"):
            prior = ImuPrior(policy())
            self.assertEqual(prior.process(sample(**{field: False}), 10_000_000_000)[1],
                             "required_imu_component_unavailable")

    def test_frame_shape_stamp_and_clock_must_be_explicit(self):
        prior = ImuPrior(policy())
        for value, now, reason in (
            (replace(sample(), frame="imu_link"), 10_000_000_000, "input_frame_mismatch"),
            (replace(sample(), quaternion=(1.,)), 10_000_000_000, "invalid_imu_shape"),
            (replace(sample(), stamp_ns=0), 10_000_000_000, "invalid_stamp"),
            (sample(), 0, "clock_unavailable")):
            self.assertEqual(prior.process(value, now)[1], reason)

    def test_age_and_future_boundary_are_not_heartbeat_freshness(self):
        self.assertEqual(ImuPrior(policy()).process(sample(), 10_200_000_000)[1], "measurement_expired")
        self.assertEqual(ImuPrior(policy()).process(sample(), 10_199_999_999)[1], "calibrating")
        self.assertEqual(ImuPrior(policy()).process(sample(10_020_000_000), 10_000_000_000)[1], "calibrating")
        self.assertEqual(ImuPrior(policy()).process(sample(10_020_000_001), 10_000_000_000)[1], "measurement_in_future")

    def test_duplicate_or_reordered_sample_does_not_destroy_valid_integration(self):
        prior = calibrated()
        prior.process(sample(10_200_000_000, a=(1., 0., 9.806)), 10_200_000_000)
        vel = prior.vel.copy()
        for stamp in (10_200_000_000, 10_190_000_000, 10_000_000_000):
            self.assertEqual(prior.process(sample(stamp), 10_210_000_000)[1], "measurement_not_newer")
            np.testing.assert_array_equal(prior.vel, vel)
        value, _ = prior.process(sample(10_250_000_000, a=(1., 0., 9.806)), 10_250_000_000)
        self.assertAlmostEqual(value["position"][0], .0075)

    def test_gap_stops_velocity_and_uses_new_baseline_without_republishing_old_state(self):
        prior = calibrated()
        prior.process(sample(10_200_000_000, a=(1., 0., 9.806)), 10_200_000_000)
        position = prior.pos.copy()
        value, reason = prior.process(sample(10_500_000_000, a=(1., 0., 9.806)), 10_500_000_000)
        self.assertIsNone(value)
        self.assertEqual(reason, "integration_rebased_after_gap")
        np.testing.assert_array_equal(prior.vel, [0., 0.])
        np.testing.assert_array_equal(prior.pos, position)
        value, reason = prior.process(sample(10_550_000_000, a=(1., 0., 9.806)), 10_550_000_000)
        self.assertEqual(reason, "measurement_integrated")
        self.assertAlmostEqual(value["position"][0], position[0]+.0025)

    def test_invalid_fresh_sample_breaks_integration_and_recovers_without_nan(self):
        prior = calibrated()
        prior.process(sample(10_200_000_000, a=(1., 0., 9.806)), 10_200_000_000)
        bad = sample(10_250_000_000, a=(math.nan, 0., 9.806))
        self.assertEqual(prior.process(bad, bad.stamp_ns)[1], "nonfinite_imu")
        self.assertEqual(prior.process(sample(10_300_000_000), 10_300_000_000)[1],
                         "integration_rebased_after_gap")
        self.assertEqual(prior.process(sample(10_350_000_000), 10_350_000_000)[1],
                         "measurement_integrated")
        self.assertTrue(np.isfinite(prior.pos).all())

    def test_ros_clock_regression_latches_until_recreated(self):
        prior = calibrated()
        self.assertEqual(prior.process(sample(9_000_000_000), 9_000_000_000)[1], "clock_reset_required")
        self.assertEqual(prior.process(sample(11_000_000_000), 11_000_000_000)[1], "clock_reset_required")

    def test_status_does_not_claim_unknown_quantities_or_stationarity_proof(self):
        status = calibrated().status("fixture")
        self.assertFalse(status["stationarity_verified"])
        self.assertFalse(status["uncertainty_estimated"])
        self.assertFalse(status["covariance_propagated"])
        self.assertFalse(status["publish_tf"])
        self.assertIn("angular_velocity", status["not_estimated"])
        self.assertEqual(status["covariance_provenance"], "fixture")


class ImuPriorRosBoundaryTests(unittest.TestCase):
    def test_exact_measurement_stamp_configured_covariances_and_status(self):
        fake = fake_node()
        for i in range(5):
            fake.now_ns = 10_000_000_000+i*50_000_000
            source = message(fake.now_ns)
            before = deepcopy(source)
            ImuOdometry.on_imu(fake, source)
            self.assertEqual(source, before)
        output = fake.publisher.publish.call_args.args[0]
        self.assertEqual(fake.publisher.publish.call_count, 1)
        self.assertEqual(output.header.stamp, source.header.stamp)
        self.assertEqual(output.header.frame_id, "fixture_prior_odom")
        self.assertEqual(output.child_frame_id, "fixture_base")
        self.assertEqual(list(output.pose.covariance), fixture_values()["pose_covariance"])
        self.assertEqual(list(output.twist.covariance), fixture_values()["twist_covariance"])
        status = json.loads(fake.status_publisher.publish.call_args.args[0].data)
        self.assertTrue(status["published"])
        self.assertEqual(status["accepted_count"], 1)

    def test_canonical_stamp_sentinel_and_covariance_checks_are_enforced(self):
        cases = (
            ("invalid_stamp", lambda m: setattr(m.header.stamp, "nanosec", 1_000_000_000)),
            ("required_imu_component_unavailable", lambda m: setattr(m, "orientation_covariance", [-1.]+[0.]*8)),
            ("required_imu_component_unavailable", lambda m: setattr(m, "linear_acceleration_covariance", [-1.]+[0.]*8)),
            ("required_imu_component_unavailable", lambda m: setattr(m, "angular_velocity_covariance", [-1.]+[0.]*8)),
            ("nonfinite_input_covariance", lambda m: setattr(m, "orientation_covariance", [math.nan]*9)))
        for reason, mutate in cases:
            fake = fake_node(); source = message(); mutate(source)
            ImuOdometry.on_imu(fake, source)
            self.assertEqual(json.loads(fake.status_publisher.publish.call_args.args[0].data)["reason"], reason)
            fake.publisher.publish.assert_not_called()
