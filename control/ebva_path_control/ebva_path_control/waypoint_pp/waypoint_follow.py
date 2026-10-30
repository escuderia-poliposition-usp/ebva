"""Pure waypoint geometry/model; preserved laws, no file/ROS/display I/O."""
import math
import numpy as np
from types import SimpleNamespace

def validated_xy_reference(values):
    points = np.asarray(values, dtype=float).copy()
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 4 or (not np.isfinite(points).all()) or (not np.array_equal(points[0], points[-1])) or (len(np.unique(points[:-1], axis=0)) < 3):
        raise ValueError('Closed XY SI reference needs >=3 distinct vertices and exact repeated first point')
    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    except FloatingPointError as error:
        raise ValueError('Unrepresentable reference geometry') from error
    if not np.isfinite(lengths).all() or np.any(lengths <= 0.0):
        raise ValueError('Adjacent XY reference segments must be finite and nonzero')
    points.setflags(write=False)
    return points

def nearest_point_on_trajectory(point, trajectory):
    """
    Return the nearest point along the given piecewise linear trajectory.
    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0] ** 2 + diffs[:, 1] ** 2
    dots = np.empty((trajectory.shape[0] - 1,))
    for i in range(dots.shape[0]):
        dots[i] = np.dot(point - trajectory[i, :], diffs[i, :])
    t = dots / l2s
    t[t < 0.0] = 0.0
    t[t > 1.0] = 1.0
    projections = trajectory[:-1, :] + (t * diffs.T).T
    dists = np.empty((projections.shape[0],))
    for i in range(dists.shape[0]):
        temp = point - projections[i]
        dists[i] = np.sqrt(np.sum(temp * temp))
    min_dist_segment = np.argmin(dists)
    return (projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment)

def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
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
        end = trajectory[i + 1, :] + 1e-06
        V = np.ascontiguousarray(end - start)
        a = np.dot(V, V)
        b = 2.0 * np.dot(V, start - point)
        c = np.dot(start, start) + np.dot(point, point) - 2.0 * np.dot(start, point) - radius * radius
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
            end = trajectory[(i + 1) % trajectory.shape[0], :] + 1e-06
            V = end - start
            a = np.dot(V, V)
            b = 2.0 * np.dot(V, start - point)
            c = np.dot(start, start) + np.dot(point, point) - 2.0 * np.dot(start, point) - radius * radius
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
    return (first_p, first_i, first_t)

def get_speed_by_curvature(curvature):
    a_lat = 0.8 * 9.81
    v_max = 12.0
    v_real = math.sqrt(a_lat / (curvature + 1e-05))
    if v_real > v_max:
        return v_max
    return v_real

def get_actuation(pose_theta, lookahead_point, position, lookahead_distance, wheelbase, curvature):
    """
    Returns actuation
    """
    waypoint_y = np.dot(np.array([np.sin(-pose_theta), np.cos(-pose_theta)]), lookahead_point[0:2] - position)
    speed = get_speed_by_curvature(curvature)
    if np.abs(waypoint_y) < 1e-06:
        return (speed, 0.0)
    radius = 1.0 / (2.0 * waypoint_y / lookahead_distance ** 2)
    steering_angle = np.arctan(wheelbase / radius)
    return (speed, steering_angle)

def calculate_average_curvature_simple(wpts, i, window_size):
    """
    Calcula a curvatura média em uma janela de waypoints usando diferenças de ângulos.
    wpts: Array de waypoints (x, y).
    i: Índice do waypoint mais próximo.
    window_size: Número de waypoints a cada lado (janela total = 2*window_size + 1).
    Retorna: Curvatura média (1/radio) ou 0.0 se não for possível calcular.
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
    if np.any(lengths < 1e-06):
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

class PurePursuitPlanner:

    def __init__(self, waypoints):
        self.wheelbase = 0.15597534362552312 + 0.15875
        self.lookahead_distance = 0.5
        self.velocity_gain = 0.5
        self.conf = SimpleNamespace(wpt_xind=0, wpt_yind=1)
        self.waypoints = validated_xy_reference(waypoints)
        self.max_reacquire = 1000000000.0

    def _get_current_waypoint(self, lookahead_distance, position, theta):
        """
        Obtém o waypoint atual a seguir.

        Regra:
        - Sempre tenta achar um ponto na circunferência de lookahead.
        - Se não achar, usa diretamente o ponto projetado mais próximo
          na centerline como alvo.
        Assim, qualquer posição inicial gera um alvo válido.
        """
        wpts = np.vstack((self.waypoints[:, self.conf.wpt_xind], self.waypoints[:, self.conf.wpt_yind])).T
        (nearest_point, nearest_dist, t, i) = nearest_point_on_trajectory(position, wpts)
        (lookahead_point, i2, t2) = first_point_on_trajectory_intersecting_circle(position, lookahead_distance, wpts, i + t, wrap=True)
        if lookahead_point is not None:
            current_waypoint = np.empty((3,))
            current_waypoint[0:2] = lookahead_point
            current_waypoint[2] = 0.0
            return current_waypoint
        if nearest_dist < self.max_reacquire:
            current_waypoint = np.empty((3,))
            current_waypoint[0:2] = nearest_point
            current_waypoint[2] = 0.0
            return current_waypoint
        return None

    def plan(self, pose_x, pose_y, pose_theta, lookahead_distance, vgain, curvature=0.0):
        """
        Retorna velocidade e ângulo de esterço
        """
        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(lookahead_distance, position, pose_theta)
        if lookahead_point is None:
            return (0.0, 0.0)
        (speed, steering_angle) = get_actuation(pose_theta, lookahead_point, position, lookahead_distance, self.wheelbase, curvature)
        speed = vgain * speed
        return (speed, steering_angle)

    def command(self, x, y, yaw):
        if not all((math.isfinite(v) for v in (x, y, yaw))):
            raise ValueError('Finite planar pose required')
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            wpts = self.waypoints
            (_, _, _, index) = nearest_point_on_trajectory(np.array([x, y]), wpts)
            curvature = calculate_average_curvature_simple(wpts, index, 5)
            (speed, steer) = self.plan(x, y, yaw, self.lookahead_distance, self.velocity_gain, curvature)
        if not all((math.isfinite(float(v)) for v in (speed, steer, curvature))):
            raise ValueError('Nonfinite waypoint result')
        return (float(speed), float(steer))
