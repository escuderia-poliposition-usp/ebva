"""Leitura offline de trajetória Odometry de uma rosbag2."""
import argparse
from pathlib import Path

from .output_paths import output_path


def read_trajectory(bag_path, topic="/odom", storage_id="sqlite3"):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get(topic) != "nav_msgs/msg/Odometry":
        raise ValueError("O tópico deve existir e conter nav_msgs/msg/Odometry: " + topic)
    message_type = get_message(types[topic])
    xs, ys = [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            message = deserialize_message(data, message_type)
            xs.append(message.pose.pose.position.x)
            ys.append(message.pose.pose.position.y)
    if not xs:
        raise ValueError("A bag não contém amostras no tópico escolhido")
    return xs, ys


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("-o", "--output", required=True, help="Figura de saída nova (.pdf, .png, .svg)")
    args = parser.parse_args(argv)
    target = output_path(args.output)
    if target.suffix.lower() not in {".pdf", ".png", ".svg"}:
        parser.error("--output deve terminar em .pdf, .png ou .svg")
    xs, ys = read_trajectory(args.bag, args.topic, args.storage_id)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    ax.plot(xs, ys)
    ax.set(xlabel="x", ylabel="y", title="Trajetória do robô")
    ax.axis("equal")
    ax.grid()
    try:
        with target.open("xb") as output:
            fig.savefig(output, format=target.suffix[1:], bbox_inches="tight")
    finally:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
