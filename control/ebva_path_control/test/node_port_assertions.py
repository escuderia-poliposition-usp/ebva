"""Inspect only the tested controller, independent of parallel DDS packages."""


def assert_controller_ports(node, publishers, subscribers):
    name, namespace = node.get_name(), node.get_namespace()
    owned_publishers = dict(node.get_publisher_names_and_types_by_node(name, namespace))
    owned_subscribers = dict(node.get_subscriber_names_and_types_by_node(name, namespace))
    for topic, types in list(owned_publishers.items()) + list(owned_subscribers.items()):
        assert 'autodrive' not in topic and '/commands/' not in topic, (
            'Actuator endpoint owned by controller: ' + topic)
        assert not any(kind in ('ebva_control_msgs/msg/ActuatorCommand',
                                'ebva_control_msgs/msg/SelectedControl') for kind in types), (
            'Actuator endpoint type owned by controller: ' + topic)
    for expected, actual in ((publishers, owned_publishers), (subscribers, owned_subscribers)):
        for topic, kind in expected.items():
            resolved = namespace.rstrip('/') + '/' + topic
            assert actual.get(resolved) == [kind], (resolved, actual)
