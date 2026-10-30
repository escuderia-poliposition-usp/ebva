# Sequências de pontos das faixas imediatamente à esquerda e à direita do
# referencial da câmera, produzidas pelo pipeline lane_segmentation.py.
#
# Referencial: x para frente, y para a direita, origem (0,0) na câmera
# (header.frame_id deve ser o frame TF correspondente, ex.: "camera_link").
# Cada sequência já vem ordenada da amostra mais próxima para a mais
# distante da câmera. Uma sequência vazia significa que aquela faixa não
# foi detectada no quadro atual.

std_msgs/Header header

geometry_msgs/Point[] left_lane
geometry_msgs/Point[] right_lane
