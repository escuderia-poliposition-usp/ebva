"""Shutdown preserves live context for backend stop and does not hide callback faults."""
import signal
from unittest.mock import patch
import pytest
from rclpy.signals import SignalHandlerOptions
from ebva_vehicle_control import nodes


class BackendDouble:
    def __init__(self, state):
        self.state = state
    def stop(self):
        assert self.state["alive"], "stop must run while its ROS context is usable"
        self.state["events"].append("stop")
        if self.state["repeat_during_stop"]:
            self.state["handlers"][signal.SIGTERM](signal.SIGTERM, None)
    def destroy_node(self):
        assert self.state["alive"]
        self.state["events"].append("destroy")


def exercise(termination=None, callback_error=None, factory_error=None,
             repeat_during_stop=False, convert_signal_error=False):
    state={"alive":False, "events":[], "handlers":{}, "restored":[],
           "repeat_during_stop":repeat_during_stop}
    originals={signum:object() for signum in (signal.SIGINT, signal.SIGTERM)}
    def install(signum, handler):
        if handler is originals[signum]:
            state["restored"].append(signum)
        else:
            state["handlers"][signum]=handler
    def init(**kwargs):
        assert kwargs["signal_handler_options"] == SignalHandlerOptions.NO
        state["alive"]=True; state["events"].append("init")
    def shutdown():
        state["events"].append("shutdown"); state["alive"]=False
    def factory():
        if factory_error:
            raise factory_error
        return BackendDouble(state)
    def once(node, timeout_sec):
        assert 0 < timeout_sec <= .1  # Silent subscriber must still service process signals.
        if termination is not None:
            try:
                state["handlers"][termination](termination, None)
            except KeyboardInterrupt:
                if convert_signal_error:
                    # Directed model of a native binding which translates an
                    # asynchronous Python exception during take_message.
                    raise RuntimeError("Unable to convert call argument to Python object")
                raise
        if callback_error:
            raise callback_error
    with patch.object(nodes.signal,"getsignal",side_effect=originals.__getitem__), \
         patch.object(nodes.signal,"signal",side_effect=install), \
         patch.object(nodes.rclpy,"init",side_effect=init), \
         patch.object(nodes.rclpy,"ok",side_effect=lambda:state["alive"]), \
         patch.object(nodes.rclpy,"spin_once",side_effect=once), \
         patch.object(nodes.rclpy,"shutdown",side_effect=shutdown), \
         patch.object(nodes,"VescBackend",BackendDouble):
        try:
            nodes.spin(factory)
        except Exception as error:
            state["error"]=error
    assert state["restored"] == [signal.SIGINT, signal.SIGTERM]
    assert not state["alive"]
    return state


@pytest.mark.parametrize("termination",[signal.SIGINT,signal.SIGTERM])
def test_process_signal_keeps_context_usable_until_backend_stop(termination):
    state=exercise(termination=termination)
    assert "error" not in state
    assert state["events"] == ["init","stop","destroy","shutdown"]


def test_real_callback_error_propagates_after_cleanup():
    error=RuntimeError("real callback defect")
    state=exercise(callback_error=error)
    assert state["error"] is error
    assert state["events"] == ["init","stop","destroy","shutdown"]


def test_constructor_failure_still_releases_context_and_signal_handlers():
    error=ValueError("invalid configuration")
    state=exercise(factory_error=error)
    assert state["error"] is error
    assert state["events"] == ["init","shutdown"]


def test_second_signal_does_not_interrupt_backend_cleanup():
    state=exercise(termination=signal.SIGINT, repeat_during_stop=True)
    assert "error" not in state
    assert state["events"] == ["init","stop","destroy","shutdown"]


@pytest.mark.parametrize("termination", [signal.SIGINT, signal.SIGTERM])
def test_signal_inside_native_take_does_not_raise_across_binding(termination):
    state = exercise(termination=termination, convert_signal_error=True)
    assert "error" not in state
    assert state["events"] == ["init", "stop", "destroy", "shutdown"]


@pytest.mark.parametrize("termination", [signal.SIGINT, signal.SIGTERM])
def test_real_callback_error_after_stop_request_still_propagates(termination):
    error = RuntimeError("real defect after signal; context remains valid")
    state = exercise(termination=termination, callback_error=error)
    assert state["error"] is error
    assert state["events"] == ["init", "stop", "destroy", "shutdown"]

