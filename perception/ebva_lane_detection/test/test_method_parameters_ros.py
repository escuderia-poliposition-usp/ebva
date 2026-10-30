"""Source-specific defaults and explicit overrides survive ROS parameter construction."""
import pytest
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from ebva_lane_detection.lane_detection_node import LaneDetectionNode
from ebva_lane_detection.segmentation_methods import METHODS, config_for_method


@pytest.mark.parametrize("method", METHODS)
def test_ros_parameters_start_from_selected_source_defaults(method):
    context = Context(); rclpy.init(args=[],context=context)
    node = None
    try:
        node = LaneDetectionNode(context=context,parameter_overrides=[Parameter("segmentation_method",value=method)])
        assert node.cfg == config_for_method(method)
        assert not hasattr(node,"tracker")
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown(context=context)


def test_explicit_parameter_overrides_preset_without_changing_other_method_defaults():
    context = Context(); rclpy.init(args=[],context=context)
    node = None
    try:
        node = LaneDetectionNode(context=context,parameter_overrides=[
            Parameter("segmentation_method",value="color_gradient_v3"),
            Parameter("yellow_s_min",value=60), Parameter("publish_debug_image",value=False)])
        assert node.cfg.yellow_s_min == 60 and node.cfg.sobel_thresh == (30,150)
        assert node.debug_pub is None
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize("parameter", [Parameter("segmentation_method",value="unknown"),
                                        Parameter("real_forward_span_m",value=float("nan"))])
def test_failed_configuration_destroys_partial_ros_node(parameter):
    destroyed = []
    class ObservedNode(LaneDetectionNode):
        def destroy_node(self):
            destroyed.append(True)
            return super().destroy_node()
    context = Context(); rclpy.init(args=[],context=context)
    try:
        with pytest.raises(ValueError): ObservedNode(context=context,parameter_overrides=[parameter])
        assert destroyed == [True]
    finally:
        rclpy.shutdown(context=context)
