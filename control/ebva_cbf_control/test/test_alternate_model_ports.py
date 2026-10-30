"""Real model contracts through ROS messages; no actuators or physical map."""
from copy import deepcopy
import json
import math
from pathlib import Path
import numpy as np
import pytest
from ebva_cbf_control.models import CbfKernel
from alternate_model_fixtures import PORTS, NOW, FRAME, PARENT, scan, velocity, nominal, port

MODELS = [p.MODEL for p in PORTS]
ORACLE = json.loads((Path(__file__).parent/"fixtures/regression.json").read_text())


@pytest.fixture(params=MODELS)
def model(request):
    return request.param


def test_atomic_exact_provenance_copy_and_distinct_model_type(model):
    p = port(model)
    s, v, n = scan(NOW-20_000_000), velocity(NOW-10_000_000), nominal(model)
    assert p.observe_scan(s, NOW, 1.) is None
    assert p.observe_velocity(v, NOW, 1.) is None
    original_s, original_v = deepcopy(s.header), deepcopy(v.header)
    s.header.frame_id, v.header.frame_id = "corrupted", "corrupted"
    output, decision = p.nominal(n, NOW, 1.)
    assert decision.status == "accepted" and isinstance(output, p.COMMAND_TYPE)
    assert output.header == n.header and output.scan_header == original_s
    assert output.velocity_header == original_v
    assert output.velocity_header.frame_id == PARENT
    assert output.velocity_child_frame_id == FRAME and output.longitudinal_velocity_mps == 1.
    expected = .05 if model == "yaw_rate_margin" else math.tan(.05)
    assert getattr(output, p.LATERAL_FIELD) == pytest.approx(expected)
    assert output.acceleration_mps2 == pytest.approx(.1)
    output.header.frame_id = "corrupted"
    assert n.header.frame_id == FRAME
    assert p.pipeline.model == model


@pytest.mark.parametrize("source", ["scan", "velocity", "nominal"])
@pytest.mark.parametrize("invalid,reason", [("time", "invalid_stamp"),
    ("old", "measurement_expired"), ("future", "measurement_in_future"),
    ("frame", "frame_mismatch")])
def test_each_original_header_rejected_before_any_lease_renewal(model, source, invalid, reason):
    p = port(model)
    make = {"scan": scan, "velocity": velocity, "nominal": lambda: nominal(model)}
    message = make[source]()
    if invalid == "time": message.header.stamp.nanosec = 1_000_000_000
    if invalid == "old": message.header.stamp.sec -= 1
    if invalid == "future": message.header.stamp.nanosec = 20_000_001
    if invalid == "frame": message.header.frame_id = "wrong"
    if source == "nominal":
        assert p.nominal(message, NOW, 1.)[1].status == reason
    else:
        assert getattr(p, "observe_"+source)(message, NOW, 1.) == reason
    assert p.pipeline.leases["speed" if source == "velocity" else source].stamp is None


def test_independent_deadlines_paused_clock_and_recovery_no_cached_heartbeat(model):
    p = port(model)
    assert p.observe_scan(scan(), NOW, 1.) is None
    assert p.observe_velocity(velocity(), NOW, 1.) is None
    assert p.nominal(nominal(model), NOW, 1.)[0] is not None
    assert p.nominal(nominal(model, NOW+1_000_000), NOW, 1.21)[1].status == "scan_unavailable_or_expired"
    assert p.observe_scan(scan(NOW+2_000_000), NOW, 1.22) is None
    assert p.nominal(nominal(model, NOW+3_000_000), NOW, 1.22)[1].status == "speed_unavailable_or_expired"
    assert p.observe_velocity(velocity(NOW+4_000_000), NOW, 1.23) is None
    assert p.nominal(nominal(model, NOW+5_000_000), NOW, 1.23)[0] is not None
    assert p.nominal(nominal(model, NOW+5_000_000), NOW, 1.24)[1].status == "nominal_stamp_rejected"


def test_invalid_scan_cannot_renew_previously_valid_data(model):
    p = port(model)
    assert p.observe_scan(scan(), NOW, 1.) is None
    bad = scan(NOW+10_000_000)
    bad.ranges[0] = float("nan")
    assert p.observe_scan(bad, NOW+10_000_000, 1.1) == "scan_rejected"
    assert p.observe_velocity(velocity(NOW+11_000_000), NOW+11_000_000, 1.21) is None
    assert p.nominal(nominal(model, NOW+12_000_000), NOW+12_000_000, 1.21)[1].status == "scan_unavailable_or_expired"


def test_clock_regression_latches_until_port_recreated(model):
    p = port(model)
    assert p.observe_scan(scan(), NOW, 1.) is None
    assert p.observe_velocity(velocity(NOW-1), NOW-1, 1.01) == "clock_reset_required"
    assert p.nominal(nominal(model, NOW+1), NOW+1, 1.02)[1].status == "clock_reset_required"


def test_velocity_parent_child_and_consumed_component_scope(model):
    p = port(model)
    v = velocity()
    v.pose.pose.orientation.w = 0.
    v.pose.covariance = [float("nan")]*36
    v.twist.twist.linear.y = float("nan")
    assert p.observe_velocity(v, NOW, 1.) is None
    v = velocity(NOW+1, float("nan"))
    assert p.observe_velocity(v, NOW+1, 1.01) == "nonfinite_longitudinal_velocity"
    v = velocity(NOW+2)
    v.child_frame_id = "lidar"
    assert p.observe_velocity(v, NOW+2, 1.02) == "velocity_child_frame_mismatch"


def test_excessive_source_skew_and_cross_model_nominal_rejected(model):
    p = port(model)
    assert p.observe_scan(scan(NOW-100_000_000), NOW, 1.) is None
    assert p.observe_velocity(velocity(), NOW, 1.) is None
    assert p.nominal(nominal(model), NOW, 1.)[1].status == "source_skew"
    other = next(m for m in MODELS if m != model)
    assert p.nominal(nominal(other, NOW+1), NOW+1, 1.01)[1].status == "nominal_type_mismatch"


@pytest.mark.parametrize("lateral,acceleration", [(float("nan"), .1), (.1, float("nan")),
                                                (float("inf"), .1)])
def test_nonfinite_nominal_never_reaches_qp_or_renews_lease(model, lateral, acceleration):
    p = port(model)
    output, decision = p.nominal(nominal(model, acceleration=acceleration, lateral=lateral), NOW, 1.)
    assert output is None and decision.status == "invalid_nominal"
    assert p.pipeline.leases["nominal"].stamp is None


@pytest.mark.parametrize("model", MODELS)
def test_infeasible_box_result_is_not_a_command(model):
    p = port(model)
    assert p.observe_scan(scan(distance=.1), NOW, 1.) is None
    assert p.observe_velocity(velocity(value=4.), NOW, 1.) is None
    output, decision = p.nominal(nominal(model, acceleration=1., lateral=.1), NOW, 1.)
    assert output is None and decision.status == "infeasible_box_bound"
    assert decision.kernel.box_contradictions and decision.kernel.max_constraint_violation > 1.


@pytest.mark.parametrize("case", [c for c in ORACLE["cases"] if c["model"] in MODELS and
    (c["model"] != "steering_tangent" or abs(math.atan(c["nominal"][1])) <= math.pi/9)],
    ids=lambda c:c["model"]+"/"+c["label"])
def test_ros_outputs_equal_original_pre_extraction_model_for_same_physical_inputs(case):
    p = port(case["model"])
    s = scan()
    s.angle_min, s.angle_max = case["angles"][0], case["angles"][-1]
    s.angle_increment = (s.angle_max-s.angle_min)/(len(case["angles"])-1)
    s.ranges = case["ranges"]
    assert p.observe_scan(s, NOW, 1.) is None
    assert p.observe_velocity(velocity(value=case["speed"]), NOW, 1.) is None
    a, u2 = case["nominal"]
    nominal_lateral = math.atan(u2) if case["model"] == "steering_tangent" else u2
    output, decision = p.nominal(nominal(case["model"], acceleration=a, lateral=nominal_lateral), NOW, 1.)
    # LaserScan.ranges is float32; captured pure inputs were float64.
    # Observed encoding error <=1.1921e-8, bounded here independently of the
    # unchanged gate (1e-6) and pure/AST equivalence suite (1e-12).
    np.testing.assert_allclose(decision.kernel.raw_control, case["control"], atol=2e-8, rtol=1e-12)
    np.testing.assert_allclose(decision.kernel.barriers, case["barriers"], atol=2e-8, rtol=1e-12)
    if case["residual"] <= 1e-6:
        assert output is not None
        np.testing.assert_allclose([output.acceleration_mps2, getattr(output,p.LATERAL_FIELD)], case["control"], atol=1e-12)
    else:
        assert output is None and decision.status in ("constraint_violation", "infeasible_box_bound")


@pytest.mark.parametrize("sign", [-1., 1.])
def test_tangent_angle_nominal_limit_and_internal_tangent_are_distinct(sign):
    p = port("steering_tangent")
    assert p.observe_scan(scan(), NOW, 1.) is None
    assert p.observe_velocity(velocity(), NOW, 1.) is None
    angle = sign*math.pi/9
    output, decision = p.nominal(nominal("steering_tangent", lateral=angle), NOW, 1.)
    assert decision.status == "accepted"
    assert output.steering_tangent == pytest.approx(math.tan(angle))
    assert output.steering_tangent != pytest.approx(angle)
    output, decision = p.nominal(nominal("steering_tangent", NOW+1, lateral=angle+sign*1e-6), NOW+1, 1.01)
    assert output is None and decision.status == "invalid_nominal"


@pytest.mark.parametrize("kwargs", [{"timeout_s":0.}, {"future_tolerance_s":.2},
    {"timeout_s":float("inf")}, {"max_skew_s":float("nan")},
    {"future_tolerance_s":-1.}])
def test_unusable_time_policy_rejected(model, kwargs):
    with pytest.raises(ValueError): port(model, **kwargs)


def test_tangent_post_filter_angle_cap_can_invalidate_accepted_qp_vector():
    # Concrete witness: the historical +/-20deg output cap is narrower than
    # the preserved QP tangent box +/-0.4, even with every inequality satisfied.
    from types import SimpleNamespace
    from ebva_cbf_control.model_kernels.steering_tangent import cbf_filter
    from ebva_cbf_control.qp import qp_hildreth
    angles = np.linspace(-math.pi/2, math.pi/2, 181)
    ranges = np.where(angles < -math.pi/12, .7, 8.)
    matrices = {}
    def solve(H, f, A, b):
        matrices.update(A=A, b=b)
        return qp_hildreth(H, f, A, b)
    control, _ = cbf_filter(SimpleNamespace(L=.33,qp_hildreth=solve), .2, ranges, angles,
                            np.array([.1,0.]))
    assert CbfKernel("steering_tangent").evaluate(.2,ranges,angles,[.1,0.]).accepted
    assert np.max(matrices["A"]@control-matrices["b"]) <= 1e-6
    assert control[1] == pytest.approx(-.4)
    projected = np.array([control[0], math.tan(-math.pi/9)])
    assert np.max(matrices["A"]@projected-matrices["b"]) > 1e-4
