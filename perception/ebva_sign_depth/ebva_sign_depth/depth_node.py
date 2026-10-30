#!/usr/bin/env python3
"""
Node ROS2 que assina um tópico de imagem RGB e um tópico de profundidade
(sincronizados no tempo), detecta placas de trânsito circulares (via
segmentar_placa.py) e imprime, para cada placa encontrada:
    - diâmetro (em pixels)
    - posição (centro, em pixels, na imagem)
    - profundidade média (metros) lida da câmera binocular, dentro da
      região onde a placa foi segmentada
    - distância estimada a partir do TAMANHO da placa (diâmetro real
      conhecido + distância focal da câmera), para comparação com a
      profundidade medida

Também publica uma imagem de debug com os contornos, diâmetros e
posições desenhados, útil para visualizar no RViz/rqt_image_view.

Requisitos:
    - ROS2 (rclpy)
    - cv_bridge
    - message_filters (sincronização aproximada de tópicos)
    - segmentar_placa.py no mesmo diretório/pacote (é importado abaixo)

Tópicos:
    Assina : rgb/color/rect/image                    (sensor_msgs/Image)
    Assina : /zed/zed_node/depth/depth_registered     (sensor_msgs/Image, 32FC1 em metros)
    Assina : /zed/zed_node/rgb/camera_info            (sensor_msgs/CameraInfo) [para obter fx]
    Publica: ~/debug_image                            (sensor_msgs/Image)  [contornos + texto]
    Publica: ~/debug_image_apenas_placas               (sensor_msgs/Image)  [fundo preto, só placas]

Uso:
    ros2 run <seu_pacote> placa_detector_node
    ou
    python3 placa_detector_node.py
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters

# Reutiliza a lógica de segmentação já desenvolvida.
# Certifique-se de que segmentar_placa.py está no mesmo diretório/pacote
# (ou instalado no PYTHONPATH) para este import funcionar.
from .depth_units import depth_metres
from .segmentation import segmentar_placa, aplicar_mascara_na_imagem


class PlacaDetectorNode(Node):

    def __init__(self):
        super().__init__("placa_detector_node")

        # --- Parâmetros (ajustáveis via ros2 param / launch) ---
        self.declare_parameter(
            "topico_entrada", "/zed/zed_node/rgb/color/rect/image"
        )
        self.declare_parameter("topico_profundidade",
                                "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("topico_camera_info",
                                "/zed/zed_node/rgb/color/rect/camera_info")
        self.declare_parameter("area_min", 300)
        self.declare_parameter("circularidade_min", 0.6)
        self.declare_parameter("fracao_branco_preto_min", 0.6)
        self.declare_parameter("publicar_debug", True)
        # Diâmetro real da placa em metros (padrão brasileiro comum:
        # placas circulares de regulamentação ~0.30 a 0.50 m; ajuste
        # conforme a placa usada).
        self.declare_parameter("diametro_real_m", 0.30)
        # Tolerância (em segundos) para casar imagem RGB e profundidade
        # que não chegam com timestamps idênticos.
        self.declare_parameter("slop_sincronizacao", 0.05)

        topico_entrada = self.get_parameter("topico_entrada").value
        topico_profundidade = self.get_parameter("topico_profundidade").value
        topico_camera_info = self.get_parameter("topico_camera_info").value
        self.area_min = self.get_parameter("area_min").value
        self.circularidade_min = self.get_parameter("circularidade_min").value
        self.fracao_branco_preto_min = self.get_parameter(
            "fracao_branco_preto_min"
        ).value
        self.publicar_debug = self.get_parameter("publicar_debug").value
        self.diametro_real_m = self.get_parameter("diametro_real_m").value
        slop = self.get_parameter("slop_sincronizacao").value

        self.bridge = CvBridge()
        self.fx = None  # distância focal (px), obtida via CameraInfo

        # --- Sincronização aproximada entre imagem RGB e profundidade ---
        self.sub_imagem = message_filters.Subscriber(self, Image, topico_entrada)
        self.sub_profundidade = message_filters.Subscriber(
            self, Image, topico_profundidade
        )
        self.sincronizador = message_filters.ApproximateTimeSynchronizer(
            [self.sub_imagem, self.sub_profundidade], queue_size=10, slop=slop
        )
        self.sincronizador.registerCallback(self.callback_imagem_profundidade)

        self.assinatura_camera_info = self.create_subscription(
            CameraInfo, topico_camera_info, self.callback_camera_info, 10
        )

        self.publicador_debug = None
        if self.publicar_debug:
            self.publicador_debug = self.create_publisher(
                Image, "~/debug_image", 10
            )
            self.publicador_apenas_placas = self.create_publisher(
                Image, "~/debug_image_apenas_placas", 10
            )

        self.get_logger().info(
            f"Node iniciado. Assinando '{topico_entrada}' + '{topico_profundidade}' "
            f"(sincronizados, slop={slop}s). CameraInfo: '{topico_camera_info}'. "
            f"Debug de imagem: {'ativado' if self.publicar_debug else 'desativado'}."
        )

    def callback_camera_info(self, msg: CameraInfo):
        # Matriz intrínseca K é 3x3 (row-major): K[0] = fx, K[4] = fy.
        if self.fx is None:
            self.get_logger().info(f"CameraInfo recebido: fx={msg.k[0]:.2f}px")
        self.fx = msg.k[0]

    def callback_imagem_profundidade(self, msg_imagem: Image, msg_profundidade: Image):
        try:
            imagem_bgr = self.bridge.imgmsg_to_cv2(msg_imagem, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Falha ao converter imagem RGB: {e}")
            return

        try:
            # "passthrough" preserva o encoding original (tipicamente
            # 32FC1, profundidade em metros, com NaN/inf para inválidos).
            imagem_profundidade = self.bridge.imgmsg_to_cv2(
                msg_profundidade, desired_encoding="passthrough"
            )
            imagem_profundidade = depth_metres(imagem_profundidade, msg_profundidade.encoding)
            if imagem_profundidade.shape != imagem_bgr.shape[:2]:
                raise ValueError('RGB and registered depth dimensions differ')
        except Exception as e:
            self.get_logger().error(f"Falha ao converter imagem de profundidade: {e}")
            return

        mascara, contornos, diametros_px = segmentar_placa(
            imagem_bgr,
            area_min=self.area_min,
            circularidade_min=self.circularidade_min,
            fracao_branco_preto_min=self.fracao_branco_preto_min,
        )

        if not contornos:
            self.get_logger().info("Nenhuma placa detectada neste frame.")
        else:
            for i, (cnt, diametro) in enumerate(zip(contornos, diametros_px), start=1):
                cx, cy = centro_do_contorno(cnt)

                profundidade_media = profundidade_media_no_contorno(
                    imagem_profundidade, cnt
                )
                texto_profundidade = (
                    f"{profundidade_media:.2f}m" if profundidade_media is not None
                    else "invalida"
                )

                distancia_por_tamanho = self.estimar_distancia_por_tamanho(diametro)
                texto_distancia_tamanho = (
                    f"{distancia_por_tamanho:.2f}m" if distancia_por_tamanho is not None
                    else "N/D (sem fx/diametro)"
                )

                self.get_logger().info(
                    f"Placa {i}: diametro={diametro}px  posicao=(x={cx}, y={cy})  "
                    f"profundidade_media={texto_profundidade}  "
                    f"distancia_por_tamanho={texto_distancia_tamanho}"
                )

        if self.publicar_debug:
            imagem_debug = desenhar_debug(imagem_bgr, contornos, diametros_px)
            msg_debug = self.bridge.cv2_to_imgmsg(imagem_debug, encoding="bgr8")
            msg_debug.header = msg_imagem.header
            self.publicador_debug.publish(msg_debug)

            # Mesmas dimensões da imagem original, resto preto: útil para
            # conferir rapidamente o que a máscara está capturando.
            imagem_apenas_placas = aplicar_mascara_na_imagem(imagem_bgr, mascara)
            msg_apenas_placas = self.bridge.cv2_to_imgmsg(
                imagem_apenas_placas, encoding="bgr8"
            )
            msg_apenas_placas.header = msg_imagem.header
            self.publicador_apenas_placas.publish(msg_apenas_placas)

    def estimar_distancia_por_tamanho(self, diametro_px):
        """
        Estima a distância até a placa usando o modelo pinhole:
            distancia = (diametro_real_m * fx) / diametro_px

        Requer fx (obtido via CameraInfo) e um diametro_px > 0. Retorna
        None se alguma dessas condições não for atendida.
        """
        if (self.fx is None or not np.isfinite(self.fx) or self.fx <= 0
                or not np.isfinite(diametro_px) or diametro_px <= 0):
            return None
        return (self.diametro_real_m * self.fx) / diametro_px


def profundidade_media_no_contorno(imagem_profundidade, contorno):
    """
    Calcula a média dos valores de profundidade (metros) dentro da
    região do contorno da placa (disco preenchido, não só o bounding
    box), ignorando pixels inválidos (NaN, inf, ou <= 0 — comuns em
    câmeras binoculares quando não há match estéreo confiável).

    Retorna a profundidade média em metros, ou None se não houver
    nenhum pixel válido na região.
    """
    mascara_disco = np.zeros(imagem_profundidade.shape[:2], dtype=np.uint8)
    cv2.drawContours(mascara_disco, [contorno], -1, 255, thickness=cv2.FILLED)

    valores = imagem_profundidade[mascara_disco == 255].astype(np.float32)
    validos = valores[np.isfinite(valores) & (valores > 0)]

    if validos.size == 0:
        return None

    return float(np.mean(validos))


def centro_do_contorno(contorno):
    """
    Retorna o centro (cx, cy) em pixels do contorno, calculado a partir
    dos momentos. Cai de volta para o centro do bounding box se a área
    for degenerada (momento zero).
    """
    momentos = cv2.moments(contorno)
    if momentos["m00"] != 0:
        cx = int(momentos["m10"] / momentos["m00"])
        cy = int(momentos["m01"] / momentos["m00"])
        return cx, cy

    x, y, largura, altura = cv2.boundingRect(contorno)
    return x + largura // 2, y + altura // 2


def desenhar_debug(imagem_bgr, contornos, diametros_px):
    """
    Desenha os contornos, centros e diâmetros estimados sobre uma cópia
    da imagem de entrada, para inspeção visual (ex.: rqt_image_view).
    """
    imagem_debug = imagem_bgr.copy()
    cv2.drawContours(imagem_debug, contornos, -1, (0, 255, 0), 2)

    for cnt, diametro in zip(contornos, diametros_px):
        cx, cy = centro_do_contorno(cnt)
        cv2.circle(imagem_debug, (cx, cy), 4, (0, 255, 0), -1)
        texto = f"d={diametro}px ({cx},{cy})"
        cv2.putText(
            imagem_debug, texto, (cx - 40, cy - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )

    return imagem_debug


def main(args=None):
    rclpy.init(args=args)
    node = PlacaDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
