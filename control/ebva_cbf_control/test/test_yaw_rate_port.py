from copy import deepcopy
import math
import pytest
from ebva_cbf_control.yaw_rate_port import YawRatePort
from yaw_rate_fixtures import NOW, FRAME, PARENT, port, scan, velocity, nominal


def test_typed_pair_exact_source_headers_child_and_speed_preserved():
    p = port()
    scan_message, velocity_message, nominal_message = scan(NOW-20_000_000), velocity(NOW-10_000_000), nominal()
    assert p.observe_scan(scan_message, NOW, 1.) is None
    assert p.observe_velocity(velocity_message, NOW, 1.) is None
    original_scan_header, original_velocity_header = deepcopy(scan_message.header), deepcopy(velocity_message.header)
    scan_message.header.frame_id = "mutated"
    velocity_message.header.stamp.sec = 999
    output, decision = p.nominal(nominal_message, NOW, 1.)
    assert decision.status == "accepted"
    assert output.header == nominal_message.header
    assert output.scan_header == original_scan_header
    assert output.velocity_header == original_velocity_header
    assert output.velocity_header.frame_id == PARENT
    assert output.velocity_child_frame_id == FRAME
    assert output.longitudinal_velocity_mps == 1.
    assert output.acceleration_mps2 == pytest.approx(.1)
    assert output.yaw_rate_rps == pytest.approx(.05)
    output.header.frame_id = "other"
    assert nominal_message.header.frame_id == FRAME


@pytest.mark.parametrize("source", ["scan", "velocity", "nominal"])
def test_malformed_source_time_rejected_before_lease(source):
    p = port()
    message = {"scan": scan, "velocity": velocity, "nominal": nominal}[source]()
    message.header.stamp.nanosec = 1_000_000_000
    if source == "nominal":
        output, decision = p.nominal(message, NOW, 1.)
        assert output is None and decision.status == "invalid_stamp"
    else:
        reason = getattr(p, "observe_" + source)(message, NOW, 1.)
        assert reason == "invalid_stamp"
    lease_name = "speed" if source == "velocity" else source
    assert p.pipeline.leases[lease_name].stamp is None


@pytest.mark.parametrize("source", ["scan", "velocity", "nominal"])
@pytest.mark.parametrize("offset,expected", [(-200_000_000, "measurement_expired"),
                                           (20_000_001, "measurement_in_future")])
def test_integer_nanosecond_age_boundaries_are_enforced(source, offset, expected):
    p = port()
    message = {"scan": scan, "velocity": velocity, "nominal": nominal}[source](NOW+offset)
    if source == "nominal":
        assert p.nominal(message, NOW, 1.)[1].status == expected
    else:
        assert getattr(p, "observe_" + source)(message, NOW, 1.) == expected


@pytest.mark.parametrize("source", ["scan", "velocity", "nominal"])
def test_source_frames_are_not_relabelled(source):
    p = port()
    message = {"scan": scan, "velocity": velocity, "nominal": nominal}[source]()
    message.header.frame_id = "wrong"
    if source == "nominal":
        assert p.nominal(message, NOW, 1.)[1].status == "frame_mismatch"
    else:
        assert getattr(p, "observe_" + source)(message, NOW, 1.) == "frame_mismatch"


def test_velocity_checks_only_consumed_component_without_qualifying_odometry():
    p = port()
    message = velocity()
    message.twist.twist.linear.y = float("nan")
    message.pose.pose.orientation.w = 0.
    message.pose.covariance = [float("nan")] * 36
    message.twist.covariance = [-1.] * 36
    assert p.observe_velocity(message, NOW, 1.) is None
    message = velocity(NOW+1_000_000, float("nan"))
    assert p.observe_velocity(message, NOW+1_000_000, 1.001) == "nonfinite_longitudinal_velocity"
    message = velocity(NOW+2_000_000)
    message.child_frame_id = "camera"
    assert p.observe_velocity(message, NOW+2_000_000, 1.002) == "velocity_child_frame_mismatch"


def test_each_measurement_expires_independently_when_ros_clock_is_paused():
    p = port()
    assert p.observe_scan(scan(), NOW, 1.) is None
    assert p.observe_velocity(velocity(), NOW, 1.) is None
    assert p.nominal(nominal(), NOW, 1.)[0] is not None
    assert p.nominal(nominal(NOW+1_000_000), NOW, 1.21)[1].status == "scan_unavailable_or_expired"
    assert p.observe_scan(scan(NOW+2_000_000), NOW, 1.22) is None
    assert p.nominal(nominal(NOW+3_000_000), NOW, 1.22)[1].status == "speed_unavailable_or_expired"
    assert p.observe_velocity(velocity(NOW+4_000_000), NOW, 1.23) is None
    output, _ = p.nominal(nominal(NOW+5_000_000), NOW, 1.23)
    assert output is not None
    assert p.nominal(nominal(NOW+5_000_000), NOW, 1.24)[1].status == "nominal_stamp_rejected"


def test_regressed_ros_clock_requires_node_recreation():
    p = port()
    assert p.observe_scan(scan(), NOW, 1.) is None
    assert p.observe_velocity(velocity(NOW-1_000_000), NOW-1_000_000, 1.01) == "clock_reset_required"
    assert p.nominal(nominal(NOW+1_000_000), NOW+1_000_000, 1.02)[1].status == "clock_reset_required"


@pytest.mark.parametrize("infeasible", [False, True])
def test_ros_gate_emits_no_command_for_inadmissible_solver_result(infeasible):
    p = port()
    message = scan(distance=.1 if infeasible else 8.)
    if not infeasible:
        message.ranges = [5. if i <= 90 else .8 for i in range(181)]
    assert p.observe_scan(message, NOW, 1.) is None
    assert p.observe_velocity(velocity(value=4. if infeasible else 1.2), NOW, 1.) is None
    output, decision = p.nominal(nominal(acceleration=.3, omega=.12), NOW, 1.)
    assert output is None
    assert decision.status == ("infeasible_box_bound" if infeasible else "constraint_violation")
    assert decision.kernel.max_constraint_violation > 0


def test_fresh_inputs_with_excessive_skew_are_rejected():
    p = port()
    assert p.observe_scan(scan(NOW-100_000_000), NOW, 1.) is None
    assert p.observe_velocity(velocity(), NOW, 1.) is None
    assert p.nominal(nominal(), NOW, 1.)[1].status == "source_skew"


@pytest.mark.parametrize("geometry,point", [("", "rear_axle_center"),
    ("coincident_aligned", ""), ("transform_by_renaming", "rear_axle_center")])
def test_geometry_and_point_must_be_explicit(geometry, point):
    with pytest.raises(ValueError):
        YawRatePort(FRAME, PARENT, point, geometry)
