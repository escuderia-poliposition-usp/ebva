import math
import struct
from types import SimpleNamespace

import numpy as np

from ebva_localization.slam_map_saver import SlamMapSaver, _yaw_from_quat, stamped_base


def test_saved_occupied_cell_respects_map_origin_rotation(tmp_path):
    quaternion = SimpleNamespace(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    origin = SimpleNamespace(position=SimpleNamespace(x=10., y=20.), orientation=quaternion)
    grid = SimpleNamespace(data=[100, 0], info=SimpleNamespace(
        width=2, height=1, resolution=1., origin=origin))
    logger = SimpleNamespace(info=lambda *a: None, error=lambda *a: None, warn=lambda *a: None)
    saver = SimpleNamespace(_last_map=grid, occupied_thresh=.65, free_thresh=.25,
                            get_logger=lambda: logger)
    base = str(tmp_path / "map")
    assert SlamMapSaver._save_pcd(saver, base)
    path = tmp_path / "map.pcd"
    original = path.read_bytes()
    data = original.split(b"DATA binary\n", 1)[1]
    np.testing.assert_allclose(struct.unpack("<3f", data), (9.5, 20.5, 0.))
    assert not SlamMapSaver._save_pcd(saver, base)
    assert path.read_bytes() == original
    assert SlamMapSaver._save_grid(saver, base)
    assert (tmp_path / "map.pgm").read_bytes().endswith(bytes([0, 254]))
    assert math.isclose(_yaw_from_quat(quaternion), math.pi/2)


def test_timestamped_map_targets_are_distinct(tmp_path):
    assert stamped_base(str(tmp_path / "map")) != stamped_base(str(tmp_path / "map"))


def test_posegraph_error_response_is_not_reported_as_success(monkeypatch):
    from ebva_localization import slam_map_saver as module
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(result=255))
    client = SimpleNamespace(wait_for_service=lambda **kw: True, call_async=lambda request: future)
    logger = SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    saver = SimpleNamespace(_serialize_cli=client, get_logger=lambda: logger, executor=None)
    monkeypatch.setattr(module.rclpy, "spin_until_future_complete", lambda *a, **kw: None)
    assert not SlamMapSaver._save_posegraph(saver, "test-map")
    future.result = lambda: SimpleNamespace(result=0)
    assert SlamMapSaver._save_posegraph(saver, "test-map")
