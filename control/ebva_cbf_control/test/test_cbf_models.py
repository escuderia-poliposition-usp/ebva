from types import SimpleNamespace
import numpy as np
import pytest
from ebva_cbf_control.qp import qp_hildreth
from ebva_cbf_control.yaw_rate.cbf_headway_beams import CBF as YawRate
from ebva_cbf_control.yaw_rate_margin.cbf_headway_beams import CBF as Margin
from ebva_cbf_control.steering_tangent.cbf_headway_beams import CBF as Tangent


@pytest.mark.parametrize('nominal', [[0,0],[5,-2],[-4,2],[.5,.1]])
def test_box_projection_has_analytic_solution(nominal):
    nominal=np.array(nominal,dtype=float)
    A=np.array([[1,0],[-1,0],[0,1],[0,-1]])
    b=np.array([3,3,.4,.4])
    result=qp_hildreth(2*np.eye(2),-2*nominal,A,b)
    np.testing.assert_allclose(result,np.clip(nominal,[-3,-.4],[3,.4]),atol=1e-8)
    assert np.all(A@result <= b+1e-8)


def test_coupled_halfspace_projection():
    result=qp_hildreth(np.eye(2),np.array([-2.,-2.]),np.array([[1.,1.]]),np.array([1.]))
    np.testing.assert_allclose(result,[.5,.5],atol=1e-8)


def test_three_real_consumers_share_exact_solver():
    assert YawRate.qp_hildreth is qp_hildreth
    assert Margin.qp_hildreth is qp_hildreth
    assert Tangent.qp_hildreth is qp_hildreth


def test_margin_is_preserved_and_models_return_finite_open_space_commands():
    node=SimpleNamespace(L=.33,qp_hildreth=qp_hildreth)
    angles=np.linspace(-np.pi/2,np.pi/2,181); ranges=np.full(181,8.)
    outputs=[cls.cbf_filter(node,1.,ranges,angles,np.array([.1,.05])) for cls in [YawRate,Margin,Tangent]]
    for u,h in outputs:
        assert u.shape==(2,) and h.shape==(20,)
        assert np.isfinite(u).all() and np.isfinite(h).all()
    np.testing.assert_allclose(outputs[0][1]-outputs[1][1],.5,atol=1e-12)
    np.testing.assert_allclose(outputs[1][1],outputs[2][1],atol=1e-12)


@pytest.mark.parametrize('cls',[YawRate,Margin,Tangent])
@pytest.mark.parametrize('ranges,expected', [([float('inf'),float('-inf')],[10.,10.]),([float('nan'),2.],[10.,2.])])
def test_all_infinite_scan_no_longer_crashes_preprocessing(cls,ranges,expected):
    scan=SimpleNamespace(ranges=ranges,range_max=10.)
    assert cls.lidarPreprocessing(None,scan).ranges==expected
