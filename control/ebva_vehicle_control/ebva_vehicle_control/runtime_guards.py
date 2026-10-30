"""Runtime guards for native launch: discovery, single instance and critical exits."""
import os
from pathlib import Path
import time

ConfigError = ValueError

_lock_stream = None


def reject_running(expected, discovered):
    collisions = sorted(set(expected) & set(discovered))
    if collisions:
        raise ConfigError("Nos do stack ja estao ativos: " + ', '.join(collisions) + ". Encerre a instancia anterior antes de iniciar outra.")


def running_conflicts(expected, discovery_seconds=2.0):
    """Use a temporary discovery node; never instantiate a vehicle driver."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(args=[], context=context)
        node = Node(f"escuderia_preflight_{os.getpid()}", context=context, enable_rosout=False, start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + discovery_seconds
        discovered = set()
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
            discovered.update(namespace.rstrip('/') + '/' + name for name, namespace in node.get_node_names_and_namespaces())
        reject_running(expected, discovered)
    except ConfigError:
        raise
    except Exception as exc:
        raise ConfigError(f"Falha ao consultar o grafo ROS: {exc}") from exc
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()


def claim_stack(directory=None):
    """Keep one instance of this bringup per user and ROS domain on this host."""
    import fcntl
    global _lock_stream
    directory = Path(directory) if directory is not None else Path.home() / '.cache/ebva_bringup'
    directory.mkdir(parents=True, exist_ok=True)
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if not domain.isdecimal():
        raise ConfigError(f"ROS_DOMAIN_ID invalido: {domain}")
    path = directory / f"stack_domain_{int(domain)}.lock"
    stream = path.open('a+')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise ConfigError(f"Outra instancia deste bringup esta rodando no dominio ROS {domain}") from exc
    stream.seek(0)
    stream.truncate()
    stream.write(str(os.getpid()) + '\n')
    stream.flush()
    _lock_stream = stream
    return stream


def stop_on_exit(event, context):
    """Stop the remaining processes when any managed process exits unexpectedly."""
    from launch.actions import EmitEvent, LogInfo
    from launch.events import Shutdown
    if context.is_shutdown:
        return []
    reason = f"Processo {event.process_name} terminou (codigo {event.returncode}); encerrando o stack"
    return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
