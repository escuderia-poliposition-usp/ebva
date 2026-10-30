import math
import pytest

from ebva_autodrive.rotations import quaternion_from_euler


@pytest.mark.parametrize("angles,expected", [
    ((0., 0., 0.), (0., 0., 0., 1.)),
    ((math.pi/2, 0., 0.), (math.sqrt(.5), 0., 0., math.sqrt(.5))),
    ((0., math.pi/2, 0.), (0., math.sqrt(.5), 0., math.sqrt(.5))),
    ((0., 0., math.pi), (0., 0., 1., 0.)),
    ((math.pi/6, math.pi/4, math.pi/3),
     (.022260026714733816, .43967973954090955, .3604234056503559, .8223631719059994)),
])
def test_static_xyz_rotation_convention(angles, expected):
    assert quaternion_from_euler(*angles) == pytest.approx(expected, abs=1e-12)
