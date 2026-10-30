import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from ebva_cbf_control.models import CbfKernel, MODELS

ORACLE = json.loads((Path(__file__).parent / "fixtures/regression.json").read_text())




@pytest.mark.parametrize("case", ORACLE["cases"], ids=lambda c: c["model"] + "/" + c["label"])
def test_outputs_equal_actual_pre_extraction_ros_method(case):
    decision = CbfKernel(case["model"]).evaluate(case["speed"], case["ranges"], case["angles"], case["nominal"])
    np.testing.assert_allclose(decision.raw_control, case["control"], rtol=1e-12, atol=1e-12)
    np.testing.assert_allclose(decision.barriers, case["barriers"], rtol=1e-12, atol=1e-12)
    assert decision.max_constraint_violation == pytest.approx(case["residual"], abs=1e-12)
    assert decision.accepted == (case["residual"] <= 1e-6)
    if not decision.accepted:
        assert decision.command is None


@pytest.mark.parametrize("model", list(MODELS))
def test_proven_box_infeasibility_rejects_finite_solver_result(model):
    case = next(c for c in ORACLE["cases"] if c["model"] == model and c["label"] == "infeasible")
    out = CbfKernel(model).evaluate(case["speed"], case["ranges"], case["angles"], case["nominal"])
    assert out.status == "infeasible_box_bound"
    assert out.box_contradictions and out.command is None
    assert np.isfinite(out.raw_control).all()
    assert out.max_constraint_violation > 1.


@pytest.mark.parametrize("model", list(MODELS))
@pytest.mark.parametrize("ranges,angles,nominal,speed", [
    ([0., 1.], [-.1, .1], [0., 0.], 1.),
    ([float("nan"), 1.], [-.1, .1], [0., 0.], 1.),
    ([float("inf"), 1.], [-.1, .1], [0., 0.], 1.),
    ([1., 1.], [.1, -.1], [0., 0.], 1.),
    ([1., 1.], [-2., 2.], [0., 0.], 1.),
    ([], [], [0., 0.], 1.),
    ([1., 1.], [-.1, .1], [0.], 1.),
    ([1., 1.], [-.1, .1], [0., float("nan")], 1.),
    ([1., 1.], [-.1, .1], [0., 0.], float("nan")),
])
def test_invalid_inputs_do_not_reach_an_accepted_command(model, ranges, angles, nominal, speed):
    out = CbfKernel(model).evaluate(speed, ranges, angles, nominal)
    assert out.status == "invalid_input" and out.command is None


@pytest.mark.parametrize("model", ["missing", "", None])
def test_unknown_model_rejected(model):
    with pytest.raises(ValueError):
        CbfKernel(model)


def test_tangent_singular_numerics_rejected_without_changing_original_model():
    out = CbfKernel("steering_tangent").evaluate(1e200, [1., 1.], [-.1, .1], [0., 0.])
    assert out.status == "numerical_failure" and out.command is None


@pytest.mark.parametrize("label", ["left", "right"])
def test_feasible_qp_witness_exists_but_legacy_iteration_result_is_inadmissible(label):
    case = next(c for c in ORACLE["cases"] if c["model"] == "yaw_rate" and c["label"] == label)
    A, b = np.asarray(case["qp"]["A"]), np.asarray(case["qp"]["b"])
    witness = np.array([-3., case["control"][1]])
    assert np.max(A @ witness - b) <= 1e-12
    out = CbfKernel("yaw_rate").evaluate(case["speed"], case["ranges"], case["angles"], case["nominal"])
    assert out.status == "constraint_violation" and out.command is None
    assert out.max_constraint_violation > .005
    assert not out.box_contradictions
