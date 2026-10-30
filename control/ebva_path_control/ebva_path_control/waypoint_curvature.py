import yaml
import numpy as np
from argparse import Namespace
import math
import os

from ament_index_python.packages import get_package_share_directory


from .waypoint_pp.waypoint_follow import get_speed_by_curvature, get_actuation


def default_config_file():
    return os.path.join(get_package_share_directory("ebva_path_control"),
                        "data", "waypoint_curvature", "config_example_map.yaml")


"""
Planner Helpers
"""


def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :])
    t = dots / l2s
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0
    projections = trajectory[:-1, :] + (t * diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp * temp))
    min_dist_segment = np.argmin(dists)
    return (
        projections[min_dist_segment],
        dists[min_dist_segment],
        t[min_dist_segment],
        min_dist_segment,
    )


def first_point_on_trajectory_intersecting_circle(
    point, radius, trajectory, t=0.0, wrap=False
):
    """
    Find the first point one radius away from the given point along the trajectory.
    """
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    trajectory = np.ascontiguousarray(trajectory)

    for i in range(start_i, trajectory.shape[0] - 1):
        start = trajectory[i, :]
        end = trajectory[i + 1, :] + 1e-6
        V = np.ascontiguousarray(end - start)

        a = np.dot(V, V)
        b = 2.0 * np.dot(V, start - point)
        c = (
            np.dot(start, start)
            + np.dot(point, point)
            - 2.0 * np.dot(start, point)
            - radius * radius
        )
        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            continue
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0 * a)
        t2 = (-b + discriminant) / (2.0 * a)

        if i == start_i:
            if 0.0 <= t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if 0.0 <= t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif 0.0 <= t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif 0.0 <= t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break

    if wrap and first_p is None:
        for i in range(-1, start_i):
            start = trajectory[i % trajectory.shape[0], :]
            end = trajectory[(i + 1) % trajectory.shape[0], :] + 1e-6
            V = end - start

            a = np.dot(V, V)
            b = 2.0 * np.dot(V, start - point)
            c = (
                np.dot(start, start)
                + np.dot(point, point)
                - 2.0 * np.dot(start, point)
                - radius * radius
            )
            discriminant = b * b - 4 * a * c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0 * a)
            t2 = (-b + discriminant) / (2.0 * a)
            if 0.0 <= t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif 0.0 <= t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t








class PurePursuitPlanner:
    """
    Example Planner
    """

    def __init__(self, conf, wb):
        self.wheelbase = wb
        self.conf = conf
        self.load_waypoints(conf)
        self.max_reacquire = 20.0

    def load_waypoints(self, conf):
        """
        loads waypoints
        """
        self.waypoints = np.loadtxt(
            conf.wpt_path,
            delimiter=conf.wpt_delim,
            skiprows=conf.wpt_rowskip,
        )


    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        """
        gets the current waypoint to follow
        """
        wpts = np.vstack(
            (self.waypoints[:, self.conf.wpt_xind], self.waypoints[:, self.conf.wpt_yind])
        ).T
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory(position, wpts)
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
                position, lookahead_distance, wpts, i + t, wrap=True
            )
            if i2 is None:
                return None
            current_waypoint = np.empty((3,))
            current_waypoint[0:2] = wpts[i2, :]
            current_waypoint[2] = 0.0
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return np.append(wpts[i, :], 0.0)
        else:
            return None

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain, curvature=0.0):
        """
        gives actuation given observation
        """
        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(
            self.waypoints, lookahead_distance, position, pose_theta
        )

        if lookahead_point is None:
            return 4.0, 0.0

        speed, steering_angle = get_actuation(
            pose_theta,
            lookahead_point,
            position,
            lookahead_distance,
            self.wheelbase,
            curvature,
        )
        speed = vgain * speed
        return speed, steering_angle


def calculate_average_curvature_simple(wpts, i, window_size):
    """
    Calcula la curvatura promedio en una ventana de waypoints usando diferencias de ángulos.
    wpts: Array de waypoints (x, y).
    i: Índice del waypoint más cercano.
    window_size: Número de waypoints a cada lado (ventana total = 2*window_size + 1).
    Retorna: Curvatura promedio (1/radio) o 0.0 si no se puede calcular.
    """
    n = wpts.shape[0]
    start_idx = max(0, i - window_size)
    end_idx = min(n, i + window_size + 1)

    indices = np.arange(start_idx, end_idx) % n
    window_points = wpts[indices]

    if len(window_points) < 3:
        return 0.0

    vectors = np.zeros((len(window_points) - 1, 2))
    for j in range(len(window_points) - 1):
        vectors[j] = window_points[j + 1] - window_points[j]

    lengths = np.sqrt(np.sum(vectors ** 2, axis=1))

    if np.any(lengths < 1e-6):
        return 0.0

    directions = vectors / lengths[:, np.newaxis]

    curvatures = []
    for j in range(len(directions) - 1):
        v1 = directions[j]
        v2 = directions[j + 1]
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        curvature = angle / lengths[j]
        curvatures.append(curvature)

    if not curvatures:
        return 0.0

    avg_curvature = np.mean(curvatures)
    return avg_curvature


def quaternion_to_yaw(q):
    """
    Converte quaternion em yaw (rad).
    Espera um objeto com campos x, y, z, w.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_config_paths_and_params(conf, base_dir):
    """
    Garante que wpt_path seja absoluto e que wpt_rowskip seja um inteiro não-negativo.
    """
    if hasattr(conf, "wpt_path"):
        if not os.path.isabs(conf.wpt_path):
            conf.wpt_path = os.path.join(base_dir, conf.wpt_path)
    else:
        conf.wpt_path = os.path.join(base_dir, "center.csv")

    if not hasattr(conf, "wpt_delim"):
        conf.wpt_delim = ","

    if hasattr(conf, "wpt_rowskip"):
        try:
            conf.wpt_rowskip = max(0, int(conf.wpt_rowskip))
        except Exception:
            conf.wpt_rowskip = 0
    else:
        conf.wpt_rowskip = 0

    return conf


def calculatePurePursuit(pose, config_file=None, *, lookahead_distance=0.5,
                        velocity_gain=0.5,
                        wheelbase=0.15597534362552312 + 0.15875,
                        curvature_window=5):
    """
    Função chamada pelo nó ROS para calcular velocidade (m/s) e direção (rad)
    a partir de uma PoseWithCovarianceStamped.
    """
    work = {
        "mass": 3.463388126201571,
        "lf": 0.15597534362552312,
        "lr": 0.15875,
        "tlad": 0.5,
        "vgain": 0.5,
    }

    config_file = os.path.abspath(config_file or default_config_file())
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)

    conf = Namespace(**config)
    conf = _normalize_config_paths_and_params(conf, os.path.dirname(config_file))

    planner = PurePursuitPlanner(conf, wheelbase)

    dt = 0.01
    _ = dt  # por enquanto não usamos explicitamente laptime

    x = pose.pose.pose.position.x
    y = pose.pose.pose.position.y
    q = pose.pose.pose.orientation
    yaw = quaternion_to_yaw(q)


    wpts = np.vstack(
        (planner.waypoints[:, conf.wpt_xind], planner.waypoints[:, conf.wpt_yind])
    ).T


    _, _, _, i = nearest_point_on_trajectory(np.array([x, y]), wpts)
    curvature = calculate_average_curvature_simple(wpts, i, curvature_window)

    speed, steer = planner.plan(x, y, yaw, lookahead_distance, velocity_gain, curvature)

    next_wpt, _, _ = first_point_on_trajectory_intersecting_circle(
        np.array([x, y]), lookahead_distance, wpts, i
    )
    if next_wpt is None:
        return 0.0, 0.0

    return float(speed), float(steer)
