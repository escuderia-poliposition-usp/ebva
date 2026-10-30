"""Command authority composition checks; no serial, DDS or driver execution."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from ebva_vehicle_control.command_contract import CommandRoute, FINAL_TOPICS, NEUTRAL_POLICY, audit_command_routes, observed_command_routes, require_command_contract
from ebva_vehicle_control.command_contract import ConfigError

def declared_route(**changes):
    route = CommandRoute('declared_example', ('/controller', '/rc_authority', '/backend'), rc_authority='/rc_authority', rc_priority=True, rc_timeout_s=0.15, autonomy_timeout_s=0.2, rc_loss='stop', autonomy_loss='stop', neutral_policy=NEUTRAL_POLICY)
    return replace(route, **changes)

def codes(report):
    return {issue['code'] for issue in report['issues']}

def test_reference_reports_both_competing_writers_and_bypass():
    report = audit_command_routes(observed_command_routes(('vehicle', 'application')), actuation_required=True)
    assert report['status'] == 'incompatible'
    assert report['writers'][FINAL_TOPICS[0]] == ['/ackermann_to_vesc_node', '/rc_reader_node']
    assert {'competing_writers', 'rc_bypass', 'rc_expiry_unspecified', 'autonomy_expiry_unspecified'} <= codes(report)
    with pytest.raises(ConfigError, match='antes dos drivers'):
        require_command_contract(report)

def test_removing_one_writer_does_not_prove_rc_priority_or_watchdog():
    for (components, expected) in [(('vehicle',), 'rc_bypass'), (('application',), 'rc_expiry_unspecified')]:
        report = audit_command_routes(observed_command_routes(components), actuation_required=True)
        assert 'competing_writers' not in codes(report)
        assert expected in codes(report)
        assert not report['compatible']

def test_declared_single_owner_is_not_reported_as_hardware_qualification():
    report = audit_command_routes([declared_route()], actuation_required=True)
    assert report['compatible'] and report['status'] == 'declared'
    assert report['scope'] == 'declared_launch_routes_not_live_graph'
    require_command_contract(report)

def test_authority_named_outside_the_path_is_a_bypass():
    report = audit_command_routes([declared_route(rc_authority='/unconnected_authority')], actuation_required=True)
    assert 'rc_bypass' in codes(report)

@pytest.mark.parametrize('timeout', [None, 0, -1, float('nan'), float('inf'), True])
def test_rc_and_autonomy_need_finite_positive_expiry(timeout):
    report = audit_command_routes([declared_route(rc_timeout_s=timeout, autonomy_timeout_s=timeout)], actuation_required=True)
    assert {'rc_expiry_unspecified', 'autonomy_expiry_unspecified'} <= codes(report)

def test_two_different_owners_cannot_split_motor_and_steering():
    motor = declared_route(outputs=(FINAL_TOPICS[0],))
    servo = declared_route(name='other_backend', chain=('/rc_authority', '/other_backend'), outputs=(FINAL_TOPICS[1],))
    assert 'split_command_owner' in codes(audit_command_routes([motor, servo], actuation_required=True))

def test_all_routes_must_obey_rc_even_if_they_share_a_final_writer():
    good = declared_route()
    bypass = declared_route(name='bypass', chain=('/controller', '/backend'))
    report = audit_command_routes([good, bypass], actuation_required=True)
    assert 'competing_writers' not in codes(report)
    assert 'rc_bypass' in codes(report)

def test_missing_owner_and_unexpected_backend_are_reported():
    report = audit_command_routes([], actuation_required=True)
    assert 'missing_writer' in codes(report)
    report = audit_command_routes([declared_route(outputs=('/commands/motor/current',))], actuation_required=True)
    assert {'unsupported_output', 'missing_writer'} <= codes(report)

def test_empty_or_unresolved_routes_and_repeated_outputs_are_rejected():
    for (route, expected) in [(declared_route(chain=()), 'invalid_route'), (declared_route(chain=('relative_node',)), 'invalid_route'), (declared_route(outputs=()), 'invalid_outputs'), (declared_route(outputs=(FINAL_TOPICS[0], FINAL_TOPICS[0])), 'invalid_outputs')]:
        assert expected in codes(audit_command_routes([route], actuation_required=True))

def test_stop_and_neutral_policy_cannot_be_claimed_by_omission():
    report = audit_command_routes([declared_route(rc_loss='hold', autonomy_loss='hold', neutral_policy='resume_unconditionally')], actuation_required=True)
    assert {'rc_expiry_unspecified', 'autonomy_expiry_unspecified', 'neutral_policy_mismatch'} <= codes(report)

def test_sensor_profile_has_no_actuation_and_does_not_accept_hidden_routes():
    report = audit_command_routes([], actuation_required=False)
    assert report['status'] == 'no_actuation' and report['compatible']
    require_command_contract(report)
    assert 'unexpected_actuation' in codes(audit_command_routes([declared_route()], actuation_required=False))

def test_missing_audit_cannot_start_a_vehicle():
    with pytest.raises(ConfigError, match='ausente'):
        require_command_contract(None)
