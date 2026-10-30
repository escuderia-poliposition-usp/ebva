"""Solicitação única e limitada de SaveMap ao slam_toolbox."""
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap


def validate_output_prefix(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError("Defina output_prefix explicitamente, sem extensão")
    raw = Path(value).expanduser()
    if raw.is_symlink():
        raise FileExistsError('O prefixo já existe como link: ' + str(raw))
    prefix = raw.resolve()
    if prefix.suffix.lower() in {".pgm", ".yaml", ".yml"}:
        raise ValueError("output_prefix deve ser um prefixo sem extensão de mapa")
    if not prefix.parent.is_dir():
        raise FileNotFoundError("Diretório de saída inexistente: " + str(prefix.parent))
    for path in (prefix, Path(str(prefix) + ".pgm"), Path(str(prefix) + ".yaml")):
        if path.exists() or path.is_symlink():
            raise FileExistsError("Destino já existe: " + str(path))
    return prefix


class SaveSlamMapOnce(Node):
    def __init__(self):
        super().__init__("save_slam_map_once")
        self.declare_parameter("output_prefix", "")
        self.declare_parameter("service_name", "/slam_toolbox/save_map")
        self.declare_parameter("timeout_s", 10.0)
        self.prefix = validate_output_prefix(self.get_parameter("output_prefix").value)
        self.timeout = self.get_parameter("timeout_s").value
        self.service_name = self.get_parameter("service_name").value
        if (type(self.timeout) not in (int, float) or not math.isfinite(self.timeout)
                or self.timeout <= 0):
            raise ValueError("timeout_s deve ser finito e positivo")
        if not isinstance(self.service_name, str) or not self.service_name:
            raise ValueError("service_name deve ser explícito")
        self.client = self.create_client(SaveMap, self.service_name)
        self.result_code = None
        self.reason = "not_requested"

    def save(self):
        deadline = time.monotonic() + self.timeout
        while not self.client.service_is_ready():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.reason = "service_timeout"
                self.get_logger().error("Timeout aguardando " + self.service_name)
                return False
            self.client.wait_for_service(timeout_sec=min(.1, remaining))
        # Recheck immediately before dispatch: never request replacement of an existing map.
        validate_output_prefix(str(self.prefix))
        request = SaveMap.Request()
        request.name.data = str(self.prefix)
        future = self.client.call_async(request)
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done():
            future.cancel()
            self.reason = "response_timeout"
            self.get_logger().error("Timeout aguardando resposta de " + self.service_name)
            return False
        try:
            response = future.result()
        except Exception as error:
            self.reason = "service_error"
            self.get_logger().error("Falha no serviço: " + str(error))
            return False
        if response is None:
            self.reason = "empty_response"
            return False
        self.result_code = response.result
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            self.reason = "save_failed"
            self.get_logger().error("SaveMap retornou result=" + str(response.result))
            return False
        self.reason = "saved"
        self.get_logger().info("SaveMap confirmou sucesso para " + str(self.prefix))
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SaveSlamMapOnce()
        return 0 if node.save() else 1
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
