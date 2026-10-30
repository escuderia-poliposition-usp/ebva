"""Installed backend subprocess lifecycle under synthetic DDS load; no devices."""
import signal
import pytest
from ebva_vehicle_control import nodes


@pytest.mark.parametrize("termination", [signal.SIGINT, signal.SIGTERM])
def test_real_dds_shutdown_under_message_load_and_repeated_signals(termination):
    """Real installed backend, synthetic DDS only; never arm or open a device.

    The child extends only stop() with a short observable cleanup window, so the
    repeated signal is known to arrive during cleanup rather than after exit.
    This stress case is evidence of lifecycle behavior, not a deterministic
    reproduction of the native ARM64 conversion failure.
    """
    import os
    import select
    import subprocess
    import sys
    import time
    import uuid
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float64
    from ebva_control_msgs.msg import SelectedControl

    namespace = "/backend_shutdown_" + uuid.uuid4().hex
    child_script = '''
import time
from ebva_vehicle_control.nodes import VescBackend, spin
class LoadedBackend(VescBackend):
    received_count = 0
    def receive(self, message):
        super().receive(message)
        self.received_count += 1
        if self.received_count == 64:
            print("DDS_LOAD_READY", flush=True)
    def stop(self):
        assert self.context.ok()
        print("BACKEND_STOP_ALIVE", flush=True)
        time.sleep(.15)
        super().stop()
spin(LoadedBackend)
'''
    context, node, executor, child = Context(), None, None, None
    lines, motors = [], []
    rclpy.init(context=context)
    try:
        node = Node("shutdown_fixture", namespace=namespace, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(SelectedControl, "control/selected", nodes.qos())
        node.create_subscription(Float64, "commands/motor/speed",
                                 lambda message: motors.append(message.data), nodes.qos())
        child = subprocess.Popen(
            [sys.executable, "-u", "-c", child_script, "--ros-args",
             "-r", "__ns:=" + namespace, "-p", "enforce_graph:=false"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            env=dict(os.environ))

        def pump_until(marker, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                assert child.poll() is None, "Backend exited early: " + "".join(lines)
                message = SelectedControl()
                message.header.stamp = node.get_clock().now().to_msg()
                message.mode = SelectedControl.STOP
                message.reason = "disarmed_shutdown_fixture"
                publisher.publish(message)
                executor.spin_once(timeout_sec=.001)
                while select.select([child.stdout], [], [], 0)[0]:
                    line = child.stdout.readline()
                    if not line:
                        break
                    lines.append(line)
                if any(marker in line for line in lines):
                    return
            pytest.fail("Backend never reached " + marker + ": " + "".join(lines))

        pump_until("DDS_LOAD_READY", 10.)
        assert motors and all(value == 0. for value in motors)
        child.send_signal(termination)
        pump_until("BACKEND_STOP_ALIVE", 3.)
        child.send_signal(signal.SIGTERM)
        child.send_signal(signal.SIGTERM)
        output, _ = child.communicate(timeout=3.)
        lines.append(output)
        assert child.returncode == 0, "".join(lines)
        assert sum(line.count("BACKEND_STOP_ALIVE") for line in lines) == 1
        assert "Traceback" not in "".join(lines)
        assert all(value == 0. for value in motors)
    finally:
        if child is not None:
            if child.poll() is None:
                child.kill()
                child.communicate(timeout=3.)
            if child.stdout is not None:
                child.stdout.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
