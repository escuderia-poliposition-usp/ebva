"""Select preserved CBF filters; serial readers and VESC bridges are retired."""
MODELS = ("yaw_rate", "yaw_rate_margin", "steering_tangent")


def selected_nodes(model):
    from launch_ros.actions import Node
    if model not in MODELS:
        raise ValueError("Unknown CBF model: " + str(model))
    return [Node(package="ebva_cbf_control", executable=model + "_filter",
                 name="cbf_filter", output="screen")]
