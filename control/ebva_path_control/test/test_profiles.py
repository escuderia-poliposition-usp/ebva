import pytest
from ebva_path_control.profiles import selected_nodes, METHODS


@pytest.mark.parametrize('method',[m for m in METHODS if m not in ('csv_frenet_pp','waypoint_pp','waypoint_control','lidar_frenet_pp','stanley_frenet')])
def test_one_selected_method(method):
    nodes=selected_nodes(method)
    assert len(nodes)==1 and nodes[0].node_package=='ebva_path_control'
    assert nodes[0].node_executable == method


def test_invalid_reference_prevents_composition(tmp_path):
    with pytest.raises(ValueError): selected_nodes('csv_frenet_pp')
    csv=tmp_path/'ref.csv';csv.write_text('s,e_y\n0,0\n2,.1\n')
    with pytest.raises(ValueError, match='Explicit CSV'):
        selected_nodes('csv_frenet_pp', str(csv))
    contract = dict(reference_frame_id='fixture_map', body_frame_id='fixture_rear_axle',
                    model_point='software_fixture_rear_axle', reference_contract='closed_progress_si_v1',
                    progress_origin='first_accepted_odometry_s0', progress_direction='path_order')
    assert len(selected_nodes('csv_frenet_pp', str(csv), **contract)) == 1
    with pytest.raises(ValueError): selected_nodes('unknown')


def test_waypoint_composition_requires_full_model_contract(tmp_path):
    csv = tmp_path/'closed.csv'
    csv.write_text('0,0\n3,0\n3,3\n0,3\n0,0\n')
    with pytest.raises(ValueError, match='Explicit waypoint'):
        selected_nodes('waypoint_pp', str(csv))
    params = dict(path_frame_id='fixture_odom', body_frame_id='fixture_rear_axle_flu',
                  model_point='software_fixture_rear_axle', path_contract='ordered_closed_xy_si_v1',
                  path_origin='shared_reference_frame', path_direction='csv_order',
                  pose_contract='cartesian_body_pose_si_v1')
    assert len(selected_nodes('waypoint_pp', str(csv), **params)) == 1


@pytest.mark.parametrize('mode',['carrot','stanley','pure_pursuit'])
def test_waypoint_effort_composition_contract(tmp_path,mode):
    from test_effort_contract import PARAMS
    csv=tmp_path/'ref.csv';csv.write_text('x_center,y_center\n0,0\n1,0\n2,0\n')
    with pytest.raises(ValueError):selected_nodes('waypoint_control',str(csv),mode)
    assert len(selected_nodes('waypoint_control',str(csv),mode,**PARAMS))==1


def test_lidar_effort_composition_contract():
    with pytest.raises(ValueError):selected_nodes('lidar_frenet_pp')
    assert len(selected_nodes('lidar_frenet_pp',scan_frame_id='fixture_lidar',
                              scan_contract='planar_body_flu_ranges_m_v1'))==1


def test_stanley_composition_requires_full_clearance_reference_contract():
    from test_stanley_frenet_contract import PARAMS
    with pytest.raises(ValueError):selected_nodes('stanley_frenet')
    assert len(selected_nodes('stanley_frenet',**PARAMS))==1
    with pytest.raises(ValueError):selected_nodes('stanley_frenet',**dict(PARAMS,reference_contract='cartesian'))
