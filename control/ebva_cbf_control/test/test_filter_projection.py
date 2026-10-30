"""Active filter projections match their documented output conventions."""
from types import SimpleNamespace
import math
import pytest
from ebva_cbf_control.models import legacy_filter_projection
from ebva_cbf_control.yaw_rate.cbf_headway_beams import CBF as Yaw
from ebva_cbf_control.yaw_rate_margin.cbf_headway_beams import CBF as Margin
from ebva_cbf_control.steering_tangent.cbf_headway_beams import CBF as Tangent


@pytest.mark.parametrize("model,cls", [("yaw_rate", Yaw), ("yaw_rate_margin", Margin), ("steering_tangent", Tangent)])
@pytest.mark.parametrize("control,speed", [([3., .4], 1.), ([-3., -.4], -.5), ([.1, .1], 0.)])
def test_characterization_matches_actual_legacy_publication(model, cls, control, speed):
    emitted = {}
    fake = SimpleNamespace(steeringSaturation=20 if model == "steering_tangent" else 120,
        thetaOld=0., deltaThetaMax=math.pi, maxThrotle=.24,
        steer_command=lambda value: emitted.update(steering=value),
        throttle_publisher=SimpleNamespace(publish=lambda msg: emitted.update(acceleration=msg.data)))
    angle = math.atan(control[1]) if model == "steering_tangent" else cls.yawrate_to_servo(None, control[1], speed, .33)[1]
    cls.publish_control(fake, angle, control[0])
    expected = legacy_filter_projection(model, control, speed)
    assert emitted["acceleration"] == pytest.approx(expected[0], abs=1e-7)
    assert emitted["steering"] == pytest.approx(expected[1], abs=1e-7)
