"""Persistência explícita de mapa OccupancyGrid e grafo do slam_toolbox.

Observa /map e oferece ~/save_map. Gera PGM/YAML e PCD a partir da grade;
solicita posegraph/data a /slam_toolbox/serialize_map quando o serviço responde.
Os arquivos recebem destinos únicos e são criados sem sobrescrever existentes.
map_save_path vazio desabilita persistência. O salvamento no encerramento é
de melhor esforço; a resposta do serviço distingue os artefatos disponíveis.
"""

import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from slam_toolbox.srv import SerializePoseGraph


def stamped_base(path):
    """Absolute base path with a YYYYmmdd_HHMMSS stamp appended.

    `path` is a base name (no extension) — we append .pgm/.yaml/.posegraph/.data,
    so unlike Phase 1's single-file saver we don't splitext here.
    """
    ts = time.strftime('%Y%m%d_%H%M%S', time.localtime())
    return os.path.abspath(path) + '_' + ts + '_' + str(time.time_ns())


class SlamMapSaver(Node):
    def __init__(self):
        super().__init__('slam_map_saver')

        self.declare_parameter('map_save_path', '')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.25)

        self.map_save_path = self.get_parameter('map_save_path').value
        self.occupied_thresh = float(self.get_parameter('occupied_thresh').value)
        self.free_thresh = float(self.get_parameter('free_thresh').value)

        self.get_logger().info(
            "map_save_path  : %s" %
            (self.map_save_path if self.map_save_path else '(disabled)'))
        self.get_logger().info("occupied_thresh: %.3f" % self.occupied_thresh)
        self.get_logger().info("free_thresh    : %.3f" % self.free_thresh)

        self._last_map = None

        # /map is latched (transient_local); match it so we get the last grid even
        # if it was published before we subscribed.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        self._serialization_group = ReentrantCallbackGroup()
        self._serialize_cli = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map',
            callback_group=self._serialization_group)

        self.create_service(Trigger, '~/save_map', self._on_save_request)

    def _on_map(self, msg):
        self._last_map = msg

    def _on_save_request(self, _request, response):
        if not self.map_save_path:
            response.success = False
            response.message = 'map_save_path parameter is not set'
            return response
        ok, msg = self.do_save()
        response.success = ok
        response.message = msg
        return response

    def do_save(self):
        """Write the grid (.pgm/.yaml) and request the pose-graph (.posegraph/.data).

        Returns (success, message). Grid success alone counts as success; the
        pose-graph is best-effort (the service may be gone during shutdown).
        """
        if not self.map_save_path:
            return False, 'map_save_path parameter is not set'

        base = stamped_base(self.map_save_path)
        grid_ok = self._save_grid(base)
        pcd_ok = self._save_pcd(base)
        graph_ok = self._save_posegraph(base)

        parts = []
        parts.append('grid saved' if grid_ok else 'grid NOT saved')
        parts.append('pcd saved' if pcd_ok else 'pcd NOT saved')
        parts.append('pose-graph saved' if graph_ok else 'pose-graph NOT saved')
        return (grid_ok or pcd_ok or graph_ok,
                '%s (%s)' % (base, ', '.join(parts)))

    def _save_grid(self, base):
        grid = self._last_map
        if grid is None:
            self.get_logger().warn('No /map received yet; skipping grid save')
            return False

        w = grid.info.width
        h = grid.info.height
        if w == 0 or h == 0:
            self.get_logger().warn('Empty /map (0x0); skipping grid save')
            return False

        pgm_path = base + '.pgm'
        yaml_path = base + '.yaml'

        # OccupancyGrid: data[0] is the lower-left cell, row-major in +x then +y.
        # PGM is stored top row first, so flip vertically.
        occ = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)  # default: unknown (gray)
        p = occ.astype(np.float32) / 100.0
        img[(occ >= 0) & (p > self.occupied_thresh)] = 0     # occupied -> black
        img[(occ >= 0) & (p < self.free_thresh)] = 254       # free     -> white
        img = np.flipud(img)

        try:
            with open(pgm_path, 'xb') as f:
                f.write(b'P5\n%d %d\n255\n' % (w, h))
                f.write(img.tobytes())
        except OSError as e:
            self.get_logger().error('Failed to write %s: %s' % (pgm_path, e))
            return False

        o = grid.info.origin
        yaw = _yaw_from_quat(o.orientation)
        try:
            with open(yaml_path, 'x') as f:
                f.write('image: %s\n' % os.path.basename(pgm_path))
                f.write('resolution: %.6f\n' % grid.info.resolution)
                f.write('origin: [%.6f, %.6f, %.6f]\n'
                        % (o.position.x, o.position.y, yaw))
                f.write('negate: 0\n')
                f.write('occupied_thresh: %.3f\n' % self.occupied_thresh)
                f.write('free_thresh: %.3f\n' % self.free_thresh)
        except OSError as e:
            self.get_logger().error('Failed to write %s: %s' % (yaml_path, e))
            return False

        self.get_logger().info('Saved grid (%dx%d) to %s' % (w, h, pgm_path))
        return True

    def _save_pcd(self, base):
        """Write occupied cells as a z=0 point cloud, in Phase 1's PCD format.

        Phase 1 (fusion_node.cpp) saves its map as a binary PCD and renders it with
        scripts/map_to_pdf.py. map_to_pdf.py reads PCD only, so emitting the Phase 2a
        occupancy grid as the same kind of PCD lets the identical tool render both
        phases for a fair comparison.
        """
        grid = self._last_map
        if grid is None:
            return False

        w = grid.info.width
        h = grid.info.height
        if w == 0 or h == 0:
            return False

        occ = np.asarray(grid.data, dtype=np.int16).reshape(h, w)
        p = occ.astype(np.float32) / 100.0
        rows, cols = np.nonzero((occ >= 0) & (p > self.occupied_thresh))

        res = grid.info.resolution
        o = grid.info.origin
        yaw = _yaw_from_quat(o.orientation)
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)

        # Cell center in the grid's own frame (data[0] is the lower-left cell, so no
        # vertical flip here — that flip is only for the top-down PGM image), then
        # rotate by the origin yaw and offset by the origin position.
        lx = (cols.astype(np.float32) + 0.5) * res
        ly = (rows.astype(np.float32) + 0.5) * res
        x = o.position.x + lx * cos_y - ly * sin_y
        y = o.position.y + lx * sin_y + ly * cos_y
        z = np.zeros_like(x)
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        n = pts.shape[0]
        pcd_path = base + '.pcd'
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\n'
            'FIELDS x y z\n'
            'SIZE 4 4 4\n'
            'TYPE F F F\n'
            'COUNT 1 1 1\n'
            'WIDTH %d\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            'POINTS %d\n'
            'DATA binary\n' % (n, n))
        try:
            with open(pcd_path, 'xb') as f:
                f.write(header.encode('ascii'))
                f.write(pts.tobytes())
        except OSError as e:
            self.get_logger().error('Failed to write %s: %s' % (pcd_path, e))
            return False

        self.get_logger().info('Saved %d occupied points to %s' % (n, pcd_path))
        return True

    def _save_posegraph(self, base):
        if not self._serialize_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'serialize_map service unavailable (slam_toolbox down?); '
                'pose-graph not saved. Call ~/save_map before shutdown for a '
                'guaranteed pose-graph.')
            return False

        req = SerializePoseGraph.Request()
        req.filename = base  # slam_toolbox appends .posegraph / .data
        future = self._serialize_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, executor=self.executor, timeout_sec=10.0)
        if future.done() and future.result() is not None:
            if future.result().result == 0:
                self.get_logger().info('Saved pose-graph to %s.posegraph/.data' % base)
                return True
            self.get_logger().warn('serialize_map returned error result=%s' % future.result().result)
            return False
        self.get_logger().warn('serialize_map call did not complete; pose-graph not saved')
        return False


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def main(argv=None):
    rclpy.init(args=argv)
    node = SlamMapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Salvar a grade disponível no encerramento; o grafo é de melhor esforço.
    if node.map_save_path:
        try:
            node.do_save()
        except Exception as e:  # best-effort: never crash the shutdown path
            node.get_logger().error('Shutdown save failed: %s' % e)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
