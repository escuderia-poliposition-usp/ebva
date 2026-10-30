# ebva_lane_hough

Biblioteca de métodos alternativos de percepção de faixas: Canny/Hough,
gradiente/Hough com memória, suavização EMA e rastreamento por segmentação.
Nenhum módulo abre câmera ao ser importado.

~~~python
from ebva_lane_hough.hough_canny import LaneDetector

available, lateral_error, heading_error, debug_image = LaneDetector().process(image_bgr)
~~~

hough_gradient.LaneDetector.process devolve
(detected, lateral_error, heading_error, confidence, debug_image).
Seu rastreador pode conservar geometria durante até max_missing_frames; consulte
detected/confidence antes de usar a estimativa. hough_canny devolve quatro itens
e não fornece essa confiança adicional.

As imagens são arrays BGR OpenCV. Coeficientes e erros pertencem à geometria
pixel/normalizada de cada método; não são automaticamente o contrato métrico
de ebva_lane_detection ou de controle. Um adaptador de percepção deve declarar
escala, frame, tempo e perdas antes de compor esse método com um controlador.

EMAFilter(alpha=0.35) oferece update/reset. tracked_segmentation contém LaneConfig,
LaneTracker e funções de processamento; a demonstração é uma execução explícita.
O pacote exige NumPy/OpenCV, mas não depende de um controlador ou de câmera ROS
para servir como biblioteca. Testes de imagem vazia verificam o contrato de perda;
não constituem avaliação de acurácia.
