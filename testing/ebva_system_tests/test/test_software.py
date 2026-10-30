from ebva_system_tests.resources import fixture_bytes, fixture_exists
from dataclasses import replace
import math
import pytest
from ebva_cbf_control.models import MODELS
from ebva_cbf_control.software import CbfSoftwarePipeline, SourceStamp, ScanSample, SpeedSample, YawRateNominal, SteeringAngleNominal
from ebva_system_tests.cbf_pipeline_replay import FRAME, scan, speed, nominal, run_campaign

@pytest.mark.parametrize('model', list(MODELS))
def test_headers_and_model_are_preserved(model):
    p = CbfSoftwarePipeline(model)
    assert p.observe_scan(scan(10.0), 10.02, 1.0)
    assert p.observe_speed(speed(10.01), 10.02, 1.0)
    out = p.nominal(nominal(model, 10.02), 10.02, 1.0)
    assert out.status == 'accepted'
    assert out.command.model == model
    assert out.command.validity_header == SourceStamp(10.0, FRAME)
    assert out.command.scan_header.seconds == 10.0
    assert out.command.speed_header.seconds == 10.01
    assert out.command.nominal_header.seconds == 10.02
    assert out.command.control[0] == pytest.approx(0.1)
    lateral = math.tan(0.05) if model == 'steering_tangent' else 0.05
    assert out.command.control[1] == pytest.approx(lateral)

@pytest.mark.parametrize('model', list(MODELS))
@pytest.mark.parametrize('source', ['scan', 'speed', 'nominal'])
def test_ros_age_is_not_replaced_by_reception_time(model, source):
    p = CbfSoftwarePipeline(model)
    scan_stamp = 9.7 if source == 'scan' else 10.0
    speed_stamp = 9.7 if source == 'speed' else 10.0
    assert p.observe_scan(scan(scan_stamp), 10.0, 1.0) == (source != 'scan')
    assert p.observe_speed(speed(speed_stamp), 10.0, 1.0) == (source != 'speed')
    out = p.nominal(nominal(model, 9.7 if source == 'nominal' else 10.0), 10.0, 1.0)
    assert out.command is None

@pytest.mark.parametrize('model', list(MODELS))
def test_paused_clock_monotonic_lease_and_duplicate_do_not_renew(model):
    p = CbfSoftwarePipeline(model)
    assert p.observe_scan(scan(10.0), 10.0, 1.0)
    assert p.observe_speed(speed(10.0), 10.0, 1.0)
    assert p.nominal(nominal(model, 10.0), 10.0, 1.0).command is not None
    assert not p.observe_scan(scan(10.0), 10.0, 1.19)
    assert not p.observe_speed(speed(10.0), 10.0, 1.19)
    out = p.nominal(nominal(model, 10.01), 10.0, 1.21)
    assert out.status == 'scan_unavailable_or_expired' and out.command is None

@pytest.mark.parametrize('model', list(MODELS))
def test_skew_rejected_even_with_three_fresh_inputs(model):
    p = CbfSoftwarePipeline(model)
    assert p.observe_scan(scan(10.0), 10.1, 1.0)
    assert p.observe_speed(speed(10.09), 10.1, 1.0)
    assert p.nominal(nominal(model, 10.1), 10.1, 1.0).status == 'source_skew'

@pytest.mark.parametrize('model', list(MODELS))
def test_future_frame_and_nonfinite_scan_rejected(model):
    p = CbfSoftwarePipeline(model)
    assert not p.observe_scan(scan(10.03), 10.0, 1.0)
    assert not p.observe_scan(scan(10.0, frame='other'), 10.0, 1.0)
    assert not p.observe_scan(scan(10.0, float('nan')), 10.0, 1.0)
    assert not p.observe_speed(speed(10.0, float('nan')), 10.0, 1.0)
    assert not p.observe_speed(speed(10.0, frame='odom'), 10.0, 1.0)
    assert p.leases['scan'].stamp is None
    assert p.leases['speed'].stamp is None

@pytest.mark.parametrize('model', list(MODELS))
def test_invalid_geometry_and_empty_forward_crop_rejected(model):
    p = CbfSoftwarePipeline(model)
    base = scan(10.0)
    assert not p.observe_scan(replace(base, angle_increment=-0.1), 10.0, 1.0)
    assert not p.observe_scan(replace(base, angle_max=1.0), 10.0, 1.0)
    assert not p.observe_scan(replace(base, ranges=(0.0,) * 181), 10.0, 1.0)
    assert not p.observe_scan(ScanSample(base.header, (1.0, 1.0), 2.0, 0.1, 2.1), 10.0, 1.0)

@pytest.mark.parametrize('model', list(MODELS))
def test_model_and_lateral_type_cannot_be_mixed(model):
    p = CbfSoftwarePipeline(model)
    wrong_model = 'yaw_rate_margin' if model == 'yaw_rate' else 'yaw_rate'
    assert p.nominal(nominal(wrong_model, 10.0), 10.0, 1.0).status == 'nominal_model_mismatch'
    cls = YawRateNominal if model == 'steering_tangent' else SteeringAngleNominal
    assert p.nominal(cls(SourceStamp(10.0, FRAME), model, 0.1, 0.05), 10.0, 1.0).status == 'nominal_type_mismatch'

def test_tangent_nominal_angle_limit_is_explicit():
    p = CbfSoftwarePipeline('steering_tangent')
    assert p.nominal(SteeringAngleNominal(SourceStamp(10.0, FRAME), 'steering_tangent', 0.1, 0.4), 10.0, 1.0).status == 'invalid_nominal'

def test_replay_reports_only_software_qualification():
    result = run_campaign()
    assert result['passed'] and len(result['events']) == 33
    assert not result['authority_integrated'] and (not result['physical_calibration'])
    assert not result['rosbag2']
    assert {e['model'] for e in result['events']} == set(MODELS)
