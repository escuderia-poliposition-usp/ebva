"""SaveMap Humble real com servidor fake, sem iniciar slam_toolbox."""
from pathlib import Path
from threading import Event, Thread
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from slam_toolbox.srv import SaveMap

from ebva_localization.save_slam_map_once import SaveSlamMapOnce, validate_output_prefix


@pytest.mark.parametrize("code,expected", [(0, True), (1, False), (255, False)])
def test_result_codes_and_string_request_data(tmp_path, code, expected):
    rclpy.init(args=["--ros-args", "-p", "output_prefix:=" + str(tmp_path / "map"),
                    "-p", "service_name:=/save_map_test", "-p", "timeout_s:=1.0"])
    node = SaveSlamMapOnce()
    server = Node("save_map_fake_server", use_global_arguments=False)
    names = []
    def handle(request, response):
        names.append(request.name.data)
        response.result = code
        return response
    service = server.create_service(SaveMap, "/save_map_test", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    worker = Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert node.client.wait_for_service(timeout_sec=3)
        assert node.save() is expected
        assert names == [str(tmp_path / "map")]
        assert node.result_code == code
    finally:
        executor.shutdown(timeout_sec=2)
        worker.join(timeout=2)
        node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_response_timeout_is_finite_and_never_reports_success(tmp_path):
    rclpy.init(args=["--ros-args", "-p", "output_prefix:=" + str(tmp_path / "map"),
                    "-p", "service_name:=/save_map_timeout", "-p", "timeout_s:=0.25"])
    node = SaveSlamMapOnce()
    server = Node("save_map_slow_server", use_global_arguments=False)
    received, release = Event(), Event()
    def handle(request, response):
        received.set()
        release.wait(3)
        response.result = 0
        return response
    service = server.create_service(SaveMap, "/save_map_timeout", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    worker = Thread(target=executor.spin, daemon=True)
    worker.start()
    try:
        assert node.client.wait_for_service(timeout_sec=3)
        started = time.monotonic()
        assert not node.save()
        assert received.wait(.5)
        assert node.reason == "response_timeout"
        assert time.monotonic() - started < 1.5
        assert node.result_code is None
    finally:
        release.set()
        executor.shutdown(timeout_sec=2)
        worker.join(timeout=2)
        node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def test_output_prefix_refuses_existing_map_and_empty_destination(tmp_path):
    with pytest.raises(ValueError):
        validate_output_prefix("")
    existing = tmp_path / "map.yaml"
    existing.write_text("original")
    with pytest.raises(FileExistsError):
        validate_output_prefix(str(tmp_path / "map"))
    assert existing.read_text() == "original"
    with pytest.raises(ValueError):
        validate_output_prefix(str(tmp_path / "new.pgm"))
    assert validate_output_prefix(str(tmp_path / "new")) == tmp_path / "new"


def test_output_prefix_refuses_a_broken_existing_link(tmp_path):
    prefix = tmp_path / "map"
    prefix.symlink_to(tmp_path / "missing")
    with pytest.raises(FileExistsError):
        validate_output_prefix(str(prefix))
    assert prefix.is_symlink()
