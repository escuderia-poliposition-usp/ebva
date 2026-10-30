"""Entry-point cleanup keeps the context alive and preserves real failures."""
import signal
import pytest
from ebva_cbf_control import yaw_rate_node


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_main_cleans_before_shutdown_and_propagates_callback_failure(monkeypatch, failure):
    events = []
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    def initialize(**kwargs):
        assert kwargs["signal_handler_options"] == yaw_rate_node.SignalHandlerOptions.NO
        events.append("init")
    def destroy():
        events.append("destroy")
        # Repeated process signals during cleanup cannot interrupt destruction.
        signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    def spin_once(node, timeout_sec):
        assert timeout_sec == .05
        events.append("spin")
        raise failure("actual callback failure")
    monkeypatch.setattr(yaw_rate_node.rclpy, "init", initialize)
    monkeypatch.setattr(yaw_rate_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(yaw_rate_node.rclpy, "spin_once", spin_once)
    monkeypatch.setattr(yaw_rate_node.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(yaw_rate_node, "CbfYawRateNode", lambda: type("Node", (), {"destroy_node": lambda self: destroy()})())
    if failure is RuntimeError:
        with pytest.raises(RuntimeError, match="actual callback failure"):
            yaw_rate_node.main([])
    else:
        yaw_rate_node.main([])
    assert events == ["init", "spin", "destroy", "shutdown"]
    assert all(signal.getsignal(s) == h for s, h in previous.items())
