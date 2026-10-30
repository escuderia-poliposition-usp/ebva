"""Both explicit entrypoints preserve lifecycle and true callback failures."""
import signal
import pytest
from ebva_cbf_control import alternate_model_node as module


@pytest.mark.parametrize("entrypoint,node_name", [("main_margin", "CbfYawRateMarginNode"),
                                                 ("main_tangent", "CbfSteeringTangentNode")])
@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_main_cleanup_before_shutdown_and_failure_propagation(monkeypatch, entrypoint, node_name, failure):
    events = []
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    def initialize(**kwargs):
        assert kwargs["signal_handler_options"] == module.SignalHandlerOptions.NO
        events.append("init")
    def destroy():
        events.append("destroy")
        signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
    def spin(node, timeout_sec):
        assert timeout_sec == .05
        events.append("spin")
        raise failure("actual callback failure")
    monkeypatch.setattr(module.rclpy, "init", initialize)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(module.rclpy, "spin_once", spin)
    monkeypatch.setattr(module.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(module, node_name, lambda: type("Node", (), {"destroy_node": lambda self: destroy()})())
    if failure is RuntimeError:
        with pytest.raises(RuntimeError, match="actual callback failure"):
            getattr(module,entrypoint)([])
    else:
        getattr(module,entrypoint)([])
    assert events == ["init", "spin", "destroy", "shutdown"]
    assert all(signal.getsignal(s)==handler for s,handler in previous.items())
