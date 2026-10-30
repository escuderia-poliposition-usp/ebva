"""Serviço real com backend SerializePoseGraph falso, sem SLAM ou dispositivos."""
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph
from std_srvs.srv import Trigger

from ebva_localization.slam_map_saver import SlamMapSaver


def test_save_service_processes_serialize_failure_without_deadlock(tmp_path):
    rclpy.init(args=["--ros-args", "-p", "map_save_path:=" + str(tmp_path / "map")])
    saver = SlamMapSaver()
    observer = Node("map_saver_test", use_global_arguments=False)
    calls = []
    def serialize(request, response):
        calls.append(request.filename)
        response.result = 255
        return response
    service = observer.create_service(SerializePoseGraph, "/slam_toolbox/serialize_map", serialize)
    client = observer.create_client(Trigger, "/slam_map_saver/save_map")
    executor = SingleThreadedExecutor()
    executor.add_node(saver)
    executor.add_node(observer)
    grid = OccupancyGrid()
    grid.info.width, grid.info.height = 1, 1
    grid.info.resolution = .1
    grid.info.origin.orientation.w = 1.
    grid.data = [100]
    saver._on_map(grid)
    try:
        deadline = time.monotonic() + 5
        while not client.service_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert client.service_is_ready()
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert future.done(), "A resposta do serviço ficou presa no callback group"
        response = future.result()
        assert calls
        assert response.success  # Grid/PCD were persisted even though the graph failed.
        assert "pose-graph NOT saved" in response.message
        assert len(list(tmp_path.glob("*.pgm"))) == 1
        assert len(list(tmp_path.glob("*.pcd"))) == 1
    finally:
        executor.remove_node(saver)
        executor.remove_node(observer)
        executor.shutdown()
        saver.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
