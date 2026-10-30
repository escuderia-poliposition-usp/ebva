import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float32MultiArray
from lane_msgs.msg import LaneBoundaries

from .confidence_fsm import LaneState
from ebva_control_contracts import MeasurementLease
from .lane_keeping_core import DEFAULTS, MODE_CODE, STATE_CODE, V1_COMPAT, LaneKeepingCore


# v2 (2026-10-22). La lógica vive en lane_keeping_core.py (probada en simulación offline).
# Este nodo solo traduce ROS <-> núcleo. Mismos tópicos que la v1:
#   entra  /lane_detection/lane_boundaries (lane_msgs/LaneBoundaries)
#   sale   /ackermann_cmd (remapear a /drive en el carro)
#   debug  /lane_keeping/debug  [t_rel, estado, detectado, lateral, heading, curvatura,
#                                direccion, velocidad, latched, modo, ancho_carril]
#          (los 9 primeros campos son los mismos de la v1; se agregan modo y ancho)
#   reset  ros2 topic pub --once /lane_keeping/reset std_msgs/msg/Empty
#
# Cambios principales (ver lane_keeping_core.py):
#   - error medido en el carro (lookahead_x=0.0), no 0.5 m adelante
#   - k_stanley=3.0, feedforward_gain=0.5, max_steering=0.26 (límite real del servo)
#   - sigue con UNA línea si la otra se pierde
#   - no frena en seco por un cuadro malo aislado (invalid_hold_time)
#   - rampa de aceleración y velocidad limitada por curvatura
#   - velocidad medida opcional (odom_topic)
# Para volver al comportamiento anterior sin cambiar código:  -p v1_compat:=true


class LaneKeepingNode(Node):

    def __init__(self, **kwargs):
        super().__init__('lane_keeping_node', **kwargs)

        self.declare_parameter('lane_topic', '/lane_detection/lane_boundaries')
        self.declare_parameter('odom_topic', 'none')      # 'none' = no usar velocidad medida
        self.declare_parameter('watchdog_timeout', 0.30)
        self.declare_parameter('v1_compat', False)
        self.declare_parameter('expected_lane_frame', 'camera_link')
        self.declare_parameter('command_frame', 'base_link')
        self.declare_parameter('command_topic', '/ackermann_cmd')

        core_params = {}
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        compat = bool(self.get_parameter('v1_compat').value)
        for name, default in DEFAULTS.items():
            value = self.get_parameter(name).value
            if isinstance(default, float):
                value = float(value)
            elif isinstance(default, bool):
                value = bool(value)
            core_params[name] = value
        if compat:
            core_params.update(V1_COMPAT)

        self.core = LaneKeepingCore(**core_params)

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.command_frame = self.get_parameter('command_frame').value
        if not self.command_frame:
            raise ValueError('command_frame must be explicit')
        self.observation = MeasurementLease(self.watchdog_timeout,
            self.get_parameter('expected_lane_frame').value)
        lane_topic = self.get_parameter('lane_topic').value
        odom_topic = str(self.get_parameter('odom_topic').value or '')
        if odom_topic.lower() in ('', 'none', 'null'):
            odom_topic = ''

        self.last_message_time = None
        self.last_control_time = None
        self.t0_ns = self.get_clock().now().nanoseconds

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.lane_subscriber = self.create_subscription(
            LaneBoundaries, lane_topic, self.lane_callback, qos)
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter('command_topic').value, 10)
        self.debug_publisher = self.create_publisher(
            Float32MultiArray, self.declare_parameter('debug_topic','/lane_keeping/debug').value, 10)
        self.reset_subscriber = self.create_subscription(
            Empty, self.declare_parameter('reset_service','/lane_keeping/reset').value, self.reset_callback, 10)
        if odom_topic:
            self.odom_subscriber = self.create_subscription(
                Odometry, odom_topic, self.odom_callback, 10)

        self.publish_drive_command(0.0, 0.0)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        p = self.core.p
        self.get_logger().info(
            f"lane_keeping v2 | lane={lane_topic} odom={odom_topic or '-'} | "
            f"speed={p['vehicle_speed']:.2f} k={p['k_stanley']:.2f} k_soft={p['k_soft']:.2f} "
            f"ff={p['feedforward_gain']:.2f} max_steer={p['max_steering']:.2f} "
            f"lookahead={p['lookahead_x']:.2f} | una_linea={p['single_line_fallback']} "
            f"hold={p['invalid_hold_time']:.2f}s a_lat={p['max_lateral_accel']:.1f} "
            f"rampa={p['accel_limit']:.1f} | latch_lost={p['latch_lost']}"
            + (' | MODO V1' if compat else ''))

    # ------------------------------------------------------------------

    def compute_dt(self):
        now = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = now
            return 0.0
        dt = (now.nanoseconds - self.last_control_time.nanoseconds) / 1e9
        self.last_control_time = now
        return max(dt, 0.0)

    def publish_drive_command(self, steering_angle, speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_publisher.publish(msg)

    def publish_debug(self, out):
        msg = Float32MultiArray()
        msg.data = [
            float((self.get_clock().now().nanoseconds - self.t0_ns) * 1e-9),
            STATE_CODE.get(out['state'], 0.0),
            1.0 if out['detected'] else 0.0,
            float(out['lateral']),
            float(out['heading']),
            float(out['curvature']),
            float(out['steering'] if out['steering'] is not None else 0.0),
            float(out['speed']),
            1.0 if out['latched'] else 0.0,
            MODE_CODE.get(out['mode'], 0.0),
            float(self.core.adapter.lane_width_est),
        ]
        self.debug_publisher.publish(msg)

    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        self.core.set_measured_speed(msg.twist.twist.linear.x)

    def lane_callback(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        ros_now = self.get_clock().now().nanoseconds*1e-9
        if not self.observation.observe(stamp, ros_now, time.monotonic(), msg.header.frame_id):
            self.get_logger().warn('Rejected lane frame/time; observation lease not renewed',
                                   throttle_duration_sec=1.)
            return
        self.last_message_time = self.get_clock().now()
        dt = self.compute_dt()

        out = self.core.step(msg.left_lane, msg.right_lane, dt)

        if out['steering'] is not None:
            self.publish_drive_command(out['steering'], out['speed'])
        self.publish_debug(out)

        event = out['event']
        if event == 'latch':
            self.get_logger().error(
                'Lane LOST after tracking - LATCHED SAFE STOP '
                '(ros2 topic pub --once /lane_keeping/reset std_msgs/msg/Empty)')
        elif event == 'latched':
            self.get_logger().error(
                'LATCHED STOP. Para continuar: ros2 topic pub --once /lane_keeping/reset std_msgs/msg/Empty',
                throttle_duration_sec=2.0)
        elif event == 'lost':
            self.get_logger().warn('Lane LOST - SAFE STOP', throttle_duration_sec=1.0)
        elif event == 'invalid':
            self.get_logger().warn('Invalid lane geometry - waiting for valid detection',
                                   throttle_duration_sec=1.0)
        else:
            self.get_logger().info(
                f"{out['state'].name} {out['mode']} | lateral={out['lateral']:.3f} "
                f"heading={out['heading']:.3f} curv={out['curvature']:.3f} "
                f"steer={out['steering']:.3f} speed={out['speed']:.2f}"
                + (' HOLD' if event == 'hold' else ''),
                throttle_duration_sec=0.5)

    def reset_callback(self, _msg):
        self.core.reset()
        self.observation.clear()
        self.last_control_time = None
        self.publish_drive_command(0.0, 0.0)
        self.get_logger().warn(
            'RESET manual recibido: el carro arrancara tras recovery_time de deteccion valida continua.')

    def watchdog_callback(self):
        ros_now = self.get_clock().now().nanoseconds*1e-9
        if not self.observation.fresh(ros_now, time.monotonic()):
            self.core.force_stop()
            self.publish_drive_command(0.0, 0.0)

    def destroy_node(self):
        try:
            self.publish_drive_command(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LaneKeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(3):
                node.publish_drive_command(0.0, 0.0)
        except Exception as e:
            print(f'Nao foi possivel publicar a parada: {e}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
