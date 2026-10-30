#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import csv
import os
import math
from datetime import datetime
from .output_paths import output_path


class ScanLoggerNode(Node):

    def __init__(self):
        super().__init__('scan_logger_node')

        # ── Parâmetros configuráveis via CLI ou launch ────────────────────────
        self.declare_parameter('output_dir', '')
        self.declare_parameter('filename',    '')          # vazio → nome automático
        self.declare_parameter('topic',       '/scan')
        self.declare_parameter('max_rows',    0)           # 0 = sem limite

        output_dir = self.get_parameter('output_dir').value
        filename   = self.get_parameter('filename').value
        topic      = self.get_parameter('topic').value
        self.max_rows = self.get_parameter('max_rows').value

        # ── Arquivo de saída ──────────────────────────────────────────────────
        if not filename:
            stamp    = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'scan_{stamp}.csv'

        if not output_dir:
            raise ValueError('Defina output_dir explicitamente')
        if os.path.basename(filename) != filename:
            raise ValueError('filename deve conter somente o nome do arquivo')
        self.csv_path = str(output_path(os.path.join(output_dir, filename)))

        self._file   = open(self.csv_path, 'x', newline='')
        self._writer = csv.writer(self._file)

        # Cabeçalho fixo + colunas dinâmicas (preenchidas no 1º msg)
        self._header_written = False
        self._row_count      = 0

        # ── Subscriber ────────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            LaserScan,
            topic,
            self._scan_callback,
            10,
        )

        self.get_logger().info(
            f"ScanLoggerNode iniciado.\n"
            f"  Tópico  : {topic}\n"
            f"  Arquivo : {self.csv_path}\n"
            f"  Limite  : {'sem limite' if self.max_rows == 0 else self.max_rows} linhas"
        )

    # ── Callback ──────────────────────────────────────────────────────────────
    def _scan_callback(self, msg: LaserScan):

        if self.max_rows > 0 and self._row_count >= self.max_rows:
            self.get_logger().info(
                f"Limite de {self.max_rows} linhas atingido. Encerrando...",
                once=True,
            )
            rclpy.shutdown()
            return

        n_beams = len(msg.ranges)

        # ── Escreve cabeçalho uma única vez ───────────────────────────────────
        if not self._header_written:
            range_cols = [f'range_{i}' for i in range(n_beams)]
            header = [
                # timestamp
                'time_sec',
                # metadados do LaserScan
                'angle_min_rad', 'angle_max_rad', 'angle_increment_rad',
                'time_increment_sec', 'scan_time_sec',
                'range_min_m', 'range_max_m',
                # ranges e intensities
                *range_cols,
                *[f'intensity_{i}' for i in range(len(msg.intensities))],
            ]
            self._writer.writerow(header)
            self._header_written = True
            self.get_logger().info(
                f"Cabeçalho escrito — {n_beams} feixes por scan."
            )

        # ── Monta a linha ─────────────────────────────────────────────────────
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Substitui inf/nan por string vazia para facilitar leitura posterior
        def clean(v):
            if math.isinf(v) or math.isnan(v):
                return ''
            return round(v, 6)

        ranges      = [clean(r) for r in msg.ranges]
        intensities = [clean(i) for i in msg.intensities]

        row = [
            round(time_sec, 9),
            round(msg.angle_min,       6),
            round(msg.angle_max,       6),
            round(msg.angle_increment, 9),
            round(msg.time_increment,  9),
            round(msg.scan_time,       6),
            round(msg.range_min,       4),
            round(msg.range_max,       4),
            *ranges,
            *intensities,
        ]

        self._writer.writerow(row)
        self._row_count += 1

        if self._row_count % 100 == 0:
            self._file.flush()
            self.get_logger().info(f"{self._row_count} scans registrados.")

    # ── Destrutor ─────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(
            f"Arquivo fechado — {self._row_count} scans salvos em '{self.csv_path}'"
        )
        super().destroy_node()


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ScanLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

