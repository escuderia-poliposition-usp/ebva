"""Contrato offline da ponte; importar não cria servidor ou nó ROS."""
import importlib
from types import SimpleNamespace

from ebva_autodrive import config
from ebva_autodrive import autodrive_bridge as bridge


def test_import_keeps_server_uninitialized():
    assert bridge.sio is None
    assert not hasattr(bridge, "autodrive_bridge")


def test_command_callbacks_preserve_the_autodrive_rounding_contract():
    bridge.callback_throttle_command(SimpleNamespace(data=.12345))
    bridge.callback_steering_command(SimpleNamespace(data=-.98765))
    assert bridge.autodrive.throttle_command == .123
    assert bridge.autodrive.steering_command == -.988
    bridge.callback_reset_command(SimpleNamespace(data=True))
    assert bridge.autodrive.reset_command is True


def test_configuration_keeps_named_ros_endpoints_without_attrdict():
    subscribers = {entry.name: entry for entry in config.pub_sub_dict.subscribers}
    assert subscribers["sub_throttle_command"].topic == "/autodrive/f1tenth_1/throttle_command"
    assert subscribers["sub_steering_command"].topic == "/autodrive/f1tenth_1/steering_command"
    assert len(config.pub_sub_dict.publishers) == 14
