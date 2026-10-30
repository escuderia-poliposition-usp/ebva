#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_segmentation.py
=====================

Pipeline de segmentação de faixas de rolamento para lane keeping, usando OpenCV.

Etapas do pipeline (nessa ordem):
  1) Filtros de cor + gradiente para isolar as marcações de faixa na perspectiva
     ORIGINAL da câmera (binarização).
  2) Conversão para bird's eye view (transformação de perspectiva) da máscara
     binária.
  3) Filtro de continuidade ESPACIAL (componentes conexos): remove ruído e
     mantém apenas blobs alongados/contínuos, típicos de marcações de faixa
     (inclusive tracejadas).
  4) Extração das faixas candidatas por um algoritmo de "rastreamento por
     varredura" (crawler) que segue a direção local da faixa passo a passo -
     em vez de varrer só na vertical, ele acompanha a direção real da curva,
     o que permite seguir curvas fechadas, inclusive de até ~90°.
  5) Seleção da faixa imediatamente à esquerda e da faixa imediatamente à
     direita do referencial do veículo/câmera.
  6) Conversão dos pontos de pixel (vista de pássaro) para o referencial da
     câmera em metros: x para frente, y para a direita, (0,0) na câmera.
  7) (Opcional, para uso em vídeo) Filtro de continuidade TEMPORAL entre
     quadros (classe LaneTracker), com suavização e rejeição de saltos.

Por que filtrar antes de fazer o warp (e não o contrário)?
  O threshold combina cor (HLS) e gradiente (Sobel). O gradiente depende da
  orientação/nitidez das bordas, e a interpolação da warp perspective sobre
  uma imagem colorida suaviza bordas e "borra" as marcações finas quando elas
  estão longe (poucos pixels). Fazendo o threshold primeiro (na resolução
  original, onde as marcações longínquas ainda têm mais pixels de suporte) e
  só então fazendo o warp da MÁSCARA BINÁRIA (interpolação apenas para
  reamostrar posição, sem se preocupar com textura/cor), o resultado fica mais
  limpo. Se o seu caso de uso for diferente (ex.: já filtrar em cima da vista
  de pássaro por outros motivos), basta trocar a ordem das chamadas em
  `process_frame` — o restante do pipeline não depende disso.

Dependências: apenas OpenCV e NumPy.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

debug = False


# ======================================================================
# 1. CONFIGURAÇÃO — todos os parâmetros ajustáveis do pipeline
# ======================================================================
@dataclass
class LaneConfig:
    # Explicit alternatives; defaults preserve the active v4 pipeline.
    segmentation_method: str = "road_mask_v4_80"
    road_l_max: int = 80

    # ---------------- Filtros de isolamento das faixas -----------------
    white_l_min: int = 170          #200           # HLS: luminância mínima p/ branco
    #yellow_h_range: Tuple[int, int] = (15, 35)  # HLS: faixa de matiz (Hue, 0-179) p/ amarelo
    yellow_h_range: Tuple[int, int] = (12, 38)  # HLS: faixa de matiz (Hue, 0-179) p/ amarelo
    yellow_s_min: int = 50 #80                      # HLS: saturação mínima p/ amarelo
    sobel_kernel: int = 3
    sobel_thresh: Tuple[int, int] = (10, 150)   # limiar no gradiente Sobel-x (bordas)
    use_clahe: bool = True                      # equalização adaptativa de contraste (robustez à luz)

    # ---------------- Transformação de perspectiva (bird's eye view) ---
    # Trapézio de ORIGEM (na imagem de entrada), como FRAÇÃO de largura/altura.
    # >>> AJUSTE ESTES 6 VALORES para o campo de visão da sua câmera. <<<
    src_top_y_frac: float = 0.58         # altura do topo do trapézio (linha do horizonte aprox.)
    src_bottom_y_frac: float = 1 #0.97       # altura da base do trapézio (próximo ao capô)
    top_diference = 0.1
    src_top_left_x_frac: float = 0.5 - top_diference + 0.015
    src_top_right_x_frac: float = 0.5 + top_diference
    src_bottom_left_x_frac: float = -0.13 #0.06
    src_bottom_right_x_frac: float = 1.18 # 0.94

    # Resolução da imagem de destino (bird's eye view)
    warped_width: int = 500
    warped_height: int = 700
    dst_margin_frac: float = 0.25   # margem lateral no destino (espaço p/ a faixa "abrir" em curvas)

    # ---------------- Calibração física (metros) ------------------------
    # >>> Os dois parâmetros abaixo são os mais importantes de calibrar no
    #     veículo/câmera real (ex.: usando um padrão conhecido no chão, ou
    #     medindo a distância até pontos de referência visíveis na imagem). <<<
    real_lane_width_m: float = 0.5        # largura lateral real coberta pela região interna do trapézio (na base)
    real_forward_span_m: float = 1.82    # distância longitudinal real coberta pela warp (base -> topo do trapézio)
    camera_to_roi_bottom_m: float = 0.2   # distância física da câmera até o ponto do chão na base da imagem
    # (offset lateral da câmera em relação ao eixo do veículo NÃO entra aqui:
    #  o resultado já é dado no referencial da própria câmera, por definição.)

    # Correção opcional de distorção de lente (deixe None se não tiver calibração)
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None

    # ---------------- Filtro de continuidade ESPACIAL --------------------
    morph_close_kernel: Tuple[int, int] = (7, 15)  # fecha falhas de tracejado (largura, altura)
    min_component_area: int = 40
    min_elongation: float = 1.2   # razão (lado maior/lado menor) mínima do blob p/ ser "tipo faixa"

    # ---------------- Rastreamento por varredura (robustez a curvas) -----
    hist_search_height_frac: float = 0.25   # fração inferior da imagem p/ localizar as bases das faixas
    hist_min_peak_value: int = 25
    hist_min_peak_distance_px: int = 60
    crawl_step_px: int = 18                 # passo ao longo da faixa a cada iteração
    crawl_window_half_size: int = 35        # meia-largura da janela de busca a cada passo
    crawl_max_misses: int = 4               # nº de passos consecutivos sem pixels antes de encerrar o rastro
    crawl_max_steps: int = 60
    crawl_direction_smoothing: float = 0.7  # EMA da direção do crawler (0=sem memória, 1=direção "travada")
    crawl_min_progress_frac: float = 0.3    # progresso mínimo (fração de crawl_step_px) p/ aceitar um "acerto";
                                             # evita que o crawler fique "preso" orbitando um blob estagnado
                                             # (ex.: região muito distante, borrada pela própria perspectiva)
    min_candidate_points: int = 4           # descarta rastros muito curtos (ruído)

    # ---------------- Amostragem dos pontos de saída ----------------------
    n_output_points: int = 15

    # ---------------- Filtro de continuidade TEMPORAL (uso em vídeo) ------
    temporal_alpha: float = 0.6               # peso do quadro novo no EMA (1 = sem suavização)
    temporal_max_lateral_jump_m: float = 0.6  # rejeita detecção que salte lateralmente mais que isso
    temporal_coast_frames: int = 5            # nº de quadros que repete a última detecção válida
    show_debug_plots: bool = False  # opt-in para inspeção offline



# Variáveis globais fornecidas para o trapézio
src_top_y_frac: float = 0.58
src_bottom_y_frac: float = 1.0
top_diference = 0.1
src_top_left_x_frac: float = 0.5 - top_diference + 0.015
src_top_right_x_frac: float = 0.5 + top_diference
src_bottom_left_x_frac: float = -0.13
src_bottom_right_x_frac: float = 1.18

def apply_road_roi(binary_mask):
    """
    Aplica uma máscara de Região de Interesse (ROI) em formato de trapézio 
    utilizando as coordenadas fracionárias fornecidas.
    """
    height, width = binary_mask.shape[:2]
    
    # Converte as frações em coordenadas de pixels (inteiros)
    pts = np.array([
        [int(src_top_left_x_frac * width), int(src_top_y_frac * height)],       # Topo Esquerdo
        [int(src_top_right_x_frac * width), int(src_top_y_frac * height)],     # Topo Direito
        [int(src_bottom_right_x_frac * width), int(src_bottom_y_frac * height)], # Base Direita
        [int(src_bottom_left_x_frac * width), int(src_bottom_y_frac * height)]   # Base Esquerda
    ], dtype=np.int32)
    
    # Cria uma máscara vazia do mesmo tamanho da imagem original
    roi_mask = np.zeros_like(binary_mask)
    
    # Preenche o trapézio com branco (255)
    cv2.fillPoly(roi_mask, [pts], 255)
    
    # Realiza a operação AND bit a bit para manter apenas o que está dentro do trapézio
    masked_road = cv2.bitwise_and(binary_mask, roi_mask)
    
    return masked_road


def overlay_on_mask(hls_img, mask):
    """
    Recebe uma imagem em HLS e uma máscara binária (0 ou 255).
    Retorna uma imagem BGR onde o fundo está em tons de cinza 
    e a máscara aparece destacada em vermelho.
    """
    # 1. Converte a imagem HLS para BGR (necessário para manipulação de cores)
    bgr_img = cv2.cvtColor(hls_img, cv2.COLOR_HLS2BGR)
    
    # 2. Converte a imagem BGR para tons de cinza
    gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    
    # 3. Transforma a imagem em cinza de volta para 3 canais (BGR) 
    # para podermos mesclar com canais coloridos
    gray_3ch = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
    
    # 4. Cria uma camada vermelha do mesmo tamanho da imagem
    # BGR para vermelho puro: (Azul=0, Verde=0, Vermelho=255)
    red_overlay = np.zeros_like(gray_3ch)
    red_overlay[:] = (255, 0, 0) # Vermelho em BGR
    
    # 5. Define a intensidade da transparência/mistura (alfa)
    # Aqui, onde a máscara for branca (255), misturamos o vermelho com o cinza
    alpha = 0.6  # 60% de opacidade para o vermelho (ajuste de 0.0 a 1.0 se quiser mais claro/forte)
    
    # Cria uma cópia da imagem em tons de cinza para aplicar o destaque
    output = gray_3ch.copy()
    
    # Normaliza a máscara para valores binários (0 ou 1)
    mask_bool = mask > 0
    if not np.any(mask_bool):
        return output
    
    # Aplica o vermelho misturado onde a máscara é ativa
    output[mask_bool] = cv2.addWeighted(
        gray_3ch[mask_bool], 1 - alpha, 
        red_overlay[mask_bool], alpha, 
        0
    )
    return output


def create_bottom_rectangle_mask(image, height_ratio: float = 0.4):
    """
    Cria uma máscara binária que mantém apenas um retângulo na parte inferior da imagem.
    
    Parâmetros:
    - image: A imagem de entrada (frame) de onde o shape será extraído.
    - height_ratio: Proporção da altura a ser mantida (ex: 0.4 mantém os 40% inferiores).
    
    Retorna:
    - mask: Uma matriz numpy com 255 na região mantida e 0 no restante.
    """
    # Extrai altura e largura diretamente do shape da imagem
    height, width = image.shape[:2]
    
    # Cria uma máscara preta (zeros) do mesmo tamanho da imagem
    mask = np.zeros((height, width), dtype=np.uint8)
    
    # Calcula a linha de corte superior com base na proporção
    start_y = int(height * (1.0 - height_ratio))
    
    # Preenche o retângulo inferior com branco (255)
    mask[start_y:height, 0:width] = 255
    
    return mask


def expand_mask_horizontally(mask, pixels: int = 10):
    """
    Alarga uma máscara binária horizontalmente, expandindo as bordas 
    para a esquerda e para a direita.
    
    Parâmetros:
    - mask: A máscara binária de entrada (0 ou 255).
    - pixels: Quantidade de pixels a adicionar de cada lado (ex: 10 pixels).
    
    Retorna:
    - expanded_mask: A máscara alargada.
    """
    # O kernel horizontal precisa ter altura = 1 e largura = (2 * pixels) + 1
    # Exemplo para 10 pixels: largura = (2 * 10) + 1 = 21 pixels
    kernel_width = (2 * pixels) + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, 1))
    
    # Aplica a dilatação usando apenas o kernel horizontal
    expanded_mask = cv2.dilate(mask, kernel, iterations=1)
    
    return expanded_mask

# ======================================================================
# 2. ISOLAMENTO DAS FAIXAS (cor + gradiente) — feito na perspectiva original
# ======================================================================
def binarize(bgr: np.ndarray, cfg: LaneConfig) -> np.ndarray:
    """Retorna uma máscara binária (0/255) destacando pixels prováveis de
    marcação de faixa, combinando thresholds de cor (branco/amarelo em HLS)
    e de gradiente (Sobel-x, para bordas independente da cor / sombras)."""
    if cfg.segmentation_method == "color_gradient_v3":
        return binarize_color_gradient_v3(bgr, cfg)
    if cfg.segmentation_method not in ("road_mask_v4_80", "road_mask_v4_50"):
        raise ValueError("unknown segmentation_method: " + cfg.segmentation_method)
    if cfg.show_debug_plots:
        import matplotlib.pyplot as plt

    if cfg.camera_matrix is not None:
        bgr = cv2.undistort(bgr, cfg.camera_matrix, cfg.dist_coeffs)

    if cfg.use_clahe:
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        bgr = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)


    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    white_mask = cv2.inRange(hls, (0, cfg.white_l_min, 0), (179, 255, 255))
    yellow_mask = cv2.inRange(
        hls,
        (cfg.yellow_h_range[0], 0, cfg.yellow_s_min),
        (cfg.yellow_h_range[1], 255, 255),
    )
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # Máscara pista (cores pretas) ======================================================================
    road_mask = cv2.inRange(hls, (0, 0, 0), (179, cfg.road_l_max, 50))
    # Cria um elemento estruturante (kernel retangular de 9x9 pixels)
    # Aumente ou diminua o tamanho (ex: 15x15) dependendo da resolução da sua imagem/vídeo
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))

    # Fecha buracos na pista
    road_mask_closed = cv2.morphologyEx(road_mask, cv2.MORPH_CLOSE, kernel)

    # Remove ruídos externos
    road_mask_cleaned = cv2.morphologyEx(road_mask_closed, cv2.MORPH_OPEN, kernel)

    # Encontra todos os contornos na máscara limpa
    contours, _ = cv2.findContours(road_mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Cria uma nova máscara vazia do mesmo tamanho
    pista_mask = np.zeros_like(road_mask_cleaned)

    if contours:
        # Opção A: Encontrar o maior contorno (assume que a pista é a maior área detectada)
        maior_contorno = max(contours, key=cv2.contourArea)
        
        # Define uma área mínima para garantir que não estamos pegando um ruído aleatório
        if cv2.contourArea(maior_contorno) > 5000: # Ajuste este valor conforme sua resolução
            cv2.drawContours(pista_mask, [maior_contorno], -1, 255, thickness=cv2.FILLED)

    # Agora, 'pista_mask' contém apenas a pista limpa e preenchida
    pista_mask = expand_mask_horizontally(pista_mask, 75)


    # Derivada em x (agora é o gradiente)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=cfg.sobel_kernel)
    abs_sobelx = np.abs(sobelx)
    max_valx = float(abs_sobelx.max())
    scaled = (255 * abs_sobelx / max_valx).astype(np.uint8) if max_valx > 0 else np.zeros_like(gray)
    grad_mask_x = cv2.inRange(scaled, cfg.sobel_thresh[0], cfg.sobel_thresh[1])

    # Define o tamanho do alargamento (ex: 5 pixels).
    # Aumente o 5 para deixar a linha mais grossa.
    kernel_horizontal = np.ones((15, 15), np.uint8)

    # Aplica a dilatação morfológica
    # Por padrão, ele alarga a partir do centro (ex: 2 pixels para esquerda, 2 para direita).
    # Isso é ideal para manter a faixa na posição original sem deslocá-la.
    mask_alargada = cv2.dilate(grad_mask_x, kernel_horizontal, iterations=1)

    # Derivada em y
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=cfg.sobel_kernel)
    abs_sobely = np.abs(sobely)
    max_valy = float(abs_sobely.max())
    scaled = (255 * abs_sobely / max_valy).astype(np.uint8) if max_valy > 0 else np.zeros_like(gray)
    grad_mask_y = cv2.inRange(scaled, cfg.sobel_thresh[0], cfg.sobel_thresh[1])


    if cfg.show_debug_plots:
        plt.figure("Máscara da pista")
        plt.subplot(1, 2, 1)
        plt.imshow(road_mask, cmap="gray")
        plt.title("road_mask")
        plt.axis("off")
        plt.subplot(1, 2, 2)
        #plt.imshow(pista_mask, cmap="gray")
        plt.imshow(overlay_on_mask(hls, pista_mask), cmap="gray")
        plt.title("road_mask_cleaned")
        plt.show()

        # plt.figure("Máscara do gradiente")
        # plt.subplot(1, 2, 1)
        # plt.imshow(color_mask, cmap="gray")
        # plt.title("grad_mask_x")
        # plt.axis("off")
        # plt.subplot(1, 2, 2)
        # plt.imshow(mask_alargada, cmap="gray")
        # plt.title("grad_mask_y")
        # plt.axis("off")
        # plt.tight_layout()
        # plt.show()

        plt.figure(1)
        plt.subplot(2,1,1)
        plt.title("Máscara de cores")
        plt.imshow(color_mask, cmap="Blues")
        plt.grid()
        plt.subplot(2,1,2)
        plt.title("Máscara do gradiente (alargada)")
        plt.imshow(grad_mask_x, cmap="Reds")
        #plt.imshow((0,0,255) where mask_alargada > 0, cmap="gray")
        #birdseye_view[binary_warped > 0] = (0, 0, 255)
        plt.grid()
        plt.show()

    #mascara_final = apply_road_roi(cv2.bitwise_and(color_mask, mask_alargada))

    #mascara_final = cv2.bitwise_and(color_mask, mask_alargada, create_bottom_rectangle_mask(bgr, height_ratio=0.48))
    # Opção 2: Linha única com o encadeamento correto
    mascara_final = cv2.bitwise_and(
    cv2.bitwise_and(color_mask, mask_alargada), 
    cv2.bitwise_and(create_bottom_rectangle_mask(bgr, height_ratio=0.48), pista_mask)
    )

    if cfg.show_debug_plots:
        plt.figure("Máscara final")
        #plt.imshow(overlay_on_mask(hls,mascara_final), cmap="gray")
        plt.imshow(mascara_final, cmap="gray")
        plt.show()

    return mascara_final

    
    #return cv2.bitwise_or(color_mask, grad_mask_x, grad_mask_y)


def binarize_color_gradient_v3(bgr: np.ndarray, cfg: LaneConfig) -> np.ndarray:
    """Retorna uma máscara binária (0/255) destacando pixels prováveis de
    marcação de faixa, combinando thresholds de cor (branco/amarelo em HLS)
    e de gradiente (Sobel-x, para bordas independente da cor / sombras)."""
    if cfg.camera_matrix is not None:
        bgr = cv2.undistort(bgr, cfg.camera_matrix, cfg.dist_coeffs)

    if cfg.use_clahe:
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        bgr = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

    hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    white_mask = cv2.inRange(hls, (0, cfg.white_l_min, 0), (179, 255, 255))
    yellow_mask = cv2.inRange(
        hls,
        (cfg.yellow_h_range[0], 0, cfg.yellow_s_min),
        (cfg.yellow_h_range[1], 255, 255),
    )
    color_mask = cv2.bitwise_or(white_mask, yellow_mask)

    # Derivada em x (agora é o gradiente)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=cfg.sobel_kernel)
    abs_sobelx = np.abs(sobelx)
    max_valx = float(abs_sobelx.max())
    scaled = (255 * abs_sobelx / max_valx).astype(np.uint8) if max_valx > 0 else np.zeros_like(gray)
    grad_mask_x = cv2.inRange(scaled, cfg.sobel_thresh[0], cfg.sobel_thresh[1])

    # Define o tamanho do alargamento (ex: 5 pixels).
    # Aumente o 5 para deixar a linha mais grossa.
    kernel_horizontal = np.ones((1, 15), np.uint8)

    # Aplica a dilatação morfológica
    # Por padrão, ele alarga a partir do centro (ex: 2 pixels para esquerda, 2 para direita).
    # Isso é ideal para manter a faixa na posição original sem deslocá-la.
    mask_alargada = cv2.dilate(grad_mask_x, kernel_horizontal, iterations=1)

    # Derivada em y
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=cfg.sobel_kernel)
    abs_sobely = np.abs(sobely)
    max_valy = float(abs_sobely.max())
    scaled = (255 * abs_sobely / max_valy).astype(np.uint8) if max_valy > 0 else np.zeros_like(gray)
    grad_mask_y = cv2.inRange(scaled, cfg.sobel_thresh[0], cfg.sobel_thresh[1])

    # plt.figure("Máscara do gradiente")
    # plt.subplot(1, 2, 1)
    # plt.imshow(color_mask, cmap="gray")
    # plt.title("grad_mask_x")
    # plt.axis("off")
    # plt.subplot(1, 2, 2)
    # plt.imshow(mask_alargada, cmap="gray")
    # plt.title("grad_mask_y")
    # plt.axis("off")
    # plt.tight_layout()
    # plt.show()

    # plt.figure(1)
    # plt.subplot(2,1,1)
    # plt.title("Máscara de cores")
    # plt.imshow(color_mask, cmap="Blues")
    # plt.grid()
    # plt.subplot(2,1,2)
    # plt.title("Máscara do gradiente (alargada)")
    # plt.imshow(mask_alargada, cmap="Reds")
    # #plt.imshow((0,0,255) where mask_alargada > 0, cmap="gray")
    # #birdseye_view[binary_warped > 0] = (0, 0, 255)
    # plt.grid()
    # plt.show()

    return cv2.bitwise_and(color_mask, mask_alargada)

# ======================================================================
# 3. TRANSFORMAÇÃO DE PERSPECTIVA (bird's eye view)
# ======================================================================
def build_perspective_transform(img_shape, cfg: LaneConfig):
    """Constrói M (original -> bird's eye) e Minv (bird's eye -> original)
    a partir dos parâmetros de LaneConfig. Todos os pontos de origem são
    definidos como FRAÇÃO da imagem de entrada, então funcionam para
    qualquer resolução."""
    h, w = img_shape[:2]
    src = np.float32([
        [w * cfg.src_top_left_x_frac,     h * cfg.src_top_y_frac],
        [w * cfg.src_top_right_x_frac,    h * cfg.src_top_y_frac],
        [w * cfg.src_bottom_right_x_frac, h * cfg.src_bottom_y_frac],
        [w * cfg.src_bottom_left_x_frac,  h * cfg.src_bottom_y_frac],
    ])
    dst = np.float32([
        [cfg.warped_width * cfg.dst_margin_frac,       0],
        [cfg.warped_width * (1 - cfg.dst_margin_frac), 0],
        [cfg.warped_width * (1 - cfg.dst_margin_frac), cfg.warped_height],
        [cfg.warped_width * cfg.dst_margin_frac,       cfg.warped_height],
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    return M, Minv, src, dst


def warp_to_birdseye(img: np.ndarray, M: np.ndarray, cfg: LaneConfig, border_value=0) -> np.ndarray:
    return cv2.warpPerspective(
        img, M, (cfg.warped_width, cfg.warped_height),
        flags=cv2.INTER_LINEAR, borderValue=border_value,
    )


def compute_scale_factors(cfg: LaneConfig, dst: np.ndarray) -> Tuple[float, float]:
    """metros/pixel nas direções longitudinal (x) e lateral (y) da bird's eye view."""
    dst_width_px = dst[1][0] - dst[0][0]
    ym_per_pix = cfg.real_lane_width_m / dst_width_px
    xm_per_pix = cfg.real_forward_span_m / cfg.warped_height
    return xm_per_pix, ym_per_pix


# ======================================================================
# 4. FILTRO DE CONTINUIDADE ESPACIAL (componentes conexos)
# ======================================================================
def spatial_continuity_filter(binary_warped: np.ndarray, cfg: LaneConfig) -> np.ndarray:
    """Fecha pequenas falhas (tracejados) e descarta blobs pequenos ou pouco
    alongados (ruído), mantendo apenas regiões com "cara" de marcação de
    faixa. A elongação é calculada como lado_maior/lado_menor (e não
    altura/largura), pois em curvas fechadas um segmento de faixa pode ficar
    orientado quase na horizontal na vista de pássaro."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, cfg.morph_close_kernel)
    closed = cv2.morphologyEx(binary_warped, cv2.MORPH_CLOSE, kernel)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    filtered = np.zeros_like(closed)
    for label in range(1, num_labels):  # 0 = fundo
        x, y, bw, bh, area = stats[label]
        if area < cfg.min_component_area:
            continue
        elongation = max(bw, bh) / float(min(bw, bh) + 1e-3)
        if elongation < cfg.min_elongation:
            continue
        filtered[labels == label] = 255
    return filtered


# ======================================================================
# 5. EXTRAÇÃO DE FAIXAS CANDIDATAS — crawler robusto a curvas (até 90°)
# ======================================================================
def find_histogram_bases(binary_warped: np.ndarray, cfg: LaneConfig) -> List[int]:
    """Localiza colunas candidatas a base de faixa via histograma da região
    inferior da imagem (picos locais, com distância mínima entre eles)."""
    h, w = binary_warped.shape
    search_h = int(h * cfg.hist_search_height_frac)
    strip = binary_warped[h - search_h:, :]
    histogram = np.sum(strip > 0, axis=0).astype(np.float32)

    peaks = []
    i = 1
    while i < w - 1:
        if (histogram[i] >= cfg.hist_min_peak_value
                and histogram[i] >= histogram[i - 1]
                and histogram[i] >= histogram[i + 1]):
            if all(abs(i - p) >= cfg.hist_min_peak_distance_px for p in peaks):
                peaks.append(i)
                i += cfg.hist_min_peak_distance_px
                continue
        i += 1
    return peaks


def _normalize(v: np.ndarray) -> np.ndarray:
    n = float(np.hypot(v[0], v[1]))
    if n < 1e-6:
        return np.array([0.0, -1.0])
    return v / n


def crawl_lane(binary_warped: np.ndarray, start_u: int, cfg: LaneConfig) -> np.ndarray:
    """Segue uma marcação de faixa passo a passo a partir de um ponto base,
    atualizando a DIREÇÃO local a cada passo (não apenas avançando para
    cima). Isso é o que permite acompanhar curvas fechadas, inclusive de
    até ~90°: a janela de busca pode "virar" e passar a avançar
    lateralmente em vez de sempre para cima. Pequenas falhas (tracejado)
    são toleradas via `crawl_max_misses`, "coastando" na última direção
    conhecida até reencontrar pixels ou desistir do rastro."""
    h, w = binary_warped.shape
    p = np.array([float(start_u), float(h - 1)])
    direction = np.array([0.0, -1.0])  # começa apontando "para frente" (para cima na imagem)

    points = [p.copy()]
    misses = 0
    pending: List[np.ndarray] = []

    for _ in range(cfg.crawl_max_steps):
        predicted = p + direction * cfg.crawl_step_px
        if not (0 <= predicted[0] < w and 0 <= predicted[1] < h):
            break

        r = cfg.crawl_window_half_size
        u0, v0 = predicted
        umin, umax = max(0, int(u0 - r)), min(w, int(u0 + r))
        vmin, vmax = max(0, int(v0 - r)), min(h, int(v0 + r))
        window = binary_warped[vmin:vmax, umin:umax]
        ys, xs = np.nonzero(window)

        accepted = False
        if len(xs) >= 3:
            centroid = np.array([umin + xs.mean(), vmin + ys.mean()])
            progress = float(np.hypot(*(centroid - p)))
            # exige avanço mínimo: se o centroide praticamente não se moveu, é sinal de
            # um blob estagnado (ex.: far-field borrado pela perspectiva) e não de uma
            # continuação legítima da faixa -> trata como falha, não como acerto.
            if progress >= cfg.crawl_min_progress_frac * cfg.crawl_step_px:
                accepted = True

        if accepted:
            raw_dir = _normalize(centroid - p)
            direction = _normalize(
                cfg.crawl_direction_smoothing * direction
                + (1 - cfg.crawl_direction_smoothing) * raw_dir
            )
            points.extend(pending)  # confirma pontos "coastados" que preencheram uma falha
            pending = []
            points.append(centroid)
            p = centroid
            misses = 0
        else:
            misses += 1
            p = predicted
            pending.append(p.copy())
            if misses > cfg.crawl_max_misses:
                break

    return np.array(points)


def find_lane_candidates(binary_warped: np.ndarray, cfg: LaneConfig) -> List[np.ndarray]:
    bases = find_histogram_bases(binary_warped, cfg)
    candidates = []
    for u0 in bases:
        pts = crawl_lane(binary_warped, u0, cfg)
        if len(pts) >= cfg.min_candidate_points:
            candidates.append(pts)
    return candidates


# ======================================================================
# 6. SELEÇÃO DA FAIXA IMEDIATAMENTE À ESQUERDA / À DIREITA
# ======================================================================
def pixel_to_camera_frame(u, v, cfg: LaneConfig, xm_per_pix, ym_per_pix,
                           warped_width, warped_height) -> Tuple[float, float]:
    """Converte um ponto de pixel da bird's eye view para o referencial da
    câmera: x para frente (m), y para a direita (m), (0,0) na câmera.
    Assume-se que a câmera está alinhada com o eixo longitudinal do veículo
    e que o trapézio de origem foi definido simetricamente em torno do eixo
    óptico (coluna central da imagem de entrada) — premissa razoável sem
    calibração extrínseca; ajuste manualmente se não for o caso."""
    x = (warped_height - v) * xm_per_pix + cfg.camera_to_roi_bottom_m
    y = (u - warped_width / 2.0) * ym_per_pix
    return float(x), float(y)


def select_immediate_lanes(candidates: List[np.ndarray], cfg: LaneConfig,
                            xm_per_pix, ym_per_pix, warped_width, warped_height):
    """Entre todas as faixas candidatas, retorna a que está imediatamente à
    esquerda (maior y negativo, i.e. mais perto de y=0 pela esquerda) e a
    que está imediatamente à direita (menor y positivo) do referencial,
    avaliadas no ponto mais próximo da câmera (base do rastro)."""
    left_candidate, right_candidate = None, None
    best_left_y, best_right_y = -np.inf, np.inf
    for pts in candidates:
        u0, v0 = pts[0]
        _, y0 = pixel_to_camera_frame(u0, v0, cfg, xm_per_pix, ym_per_pix, warped_width, warped_height)
        if y0 < 0 and y0 > best_left_y:
            best_left_y = y0
            left_candidate = pts
        elif y0 > 0 and y0 < best_right_y:
            best_right_y = y0
            right_candidate = pts
    return left_candidate, right_candidate


def resample_points(points: np.ndarray, n: int) -> np.ndarray:
    if len(points) <= n:
        return points
    idx = np.linspace(0, len(points) - 1, n).round().astype(int)
    return points[idx]


def candidate_to_camera_points(pts_uv: Optional[np.ndarray], cfg: LaneConfig,
                                xm_per_pix, ym_per_pix, warped_width, warped_height,
                                n_out: int) -> List[Tuple[float, float]]:
    if pts_uv is None:
        return []
    pts_uv = resample_points(pts_uv, n_out)
    return [
        pixel_to_camera_frame(u, v, cfg, xm_per_pix, ym_per_pix, warped_width, warped_height)
        for u, v in pts_uv
    ]


# ======================================================================
# 7. FILTRO DE CONTINUIDADE TEMPORAL (opcional — uso em vídeo/stream)
# ======================================================================
class LaneTracker:
    """Suaviza e mantém as faixas entre quadros consecutivos de um vídeo,
    evitando saltos bruscos por ruído pontual em um único quadro e
    "segurando" a última detecção válida por alguns quadros quando a faixa
    momentaneamente some (oclusão, marcação apagada, etc.).

    Uso típico (laço de vídeo):
        tracker = LaneTracker(cfg)
        while True:
            frame = pega_proximo_quadro()
            debug_img, left_xy, right_xy = process_frame(frame, cfg, tracker=tracker)
            manda_pontos_para_controle(left_xy, right_xy)
    """

    def __init__(self, cfg: LaneConfig):
        self.cfg = cfg
        self.left_pts: Optional[np.ndarray] = None
        self.right_pts: Optional[np.ndarray] = None
        self.left_missing = 0
        self.right_missing = 0

    @staticmethod
    def _resample_to_x(pts: np.ndarray, x_grid: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts)
        return np.interp(x_grid, pts[:, 0], pts[:, 1], left=np.nan, right=np.nan)

    def _update_side(self, prev_pts, new_pts, missing_counter):
        cfg = self.cfg
        if new_pts is None or len(new_pts) < 2:
            missing_counter += 1
            if prev_pts is not None and missing_counter <= cfg.temporal_coast_frames:
                return prev_pts, missing_counter
            return None, missing_counter

        new_pts = np.asarray(new_pts, dtype=np.float64)
        if prev_pts is None:
            return new_pts, 0

        prev_pts = np.asarray(prev_pts, dtype=np.float64)
        if abs(new_pts[0, 1] - prev_pts[0, 1]) > cfg.temporal_max_lateral_jump_m:
            missing_counter += 1
            if missing_counter <= cfg.temporal_coast_frames:
                return prev_pts, missing_counter
            return new_pts, 0  # aceita mesmo assim após várias rejeições (evita travar em erro antigo)

        x_grid = new_pts[:, 0]
        prev_y = self._resample_to_x(prev_pts, x_grid)
        alpha = cfg.temporal_alpha
        blended_y = np.where(np.isnan(prev_y), new_pts[:, 1], alpha * new_pts[:, 1] + (1 - alpha) * prev_y)
        return np.stack([x_grid, blended_y], axis=1), 0

    def update(self, left_pts, right_pts):
        self.left_pts, self.left_missing = self._update_side(self.left_pts, left_pts, self.left_missing)
        self.right_pts, self.right_missing = self._update_side(self.right_pts, right_pts, self.right_missing)
        return self.left_pts, self.right_pts


# ======================================================================
# 8. VISUALIZAÇÃO
# ======================================================================
def draw_debug_image(binary_warped, candidates, left_uv, right_uv) -> np.ndarray:
    vis = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)
    vis = (vis.astype(np.float32) * 0.5).astype(np.uint8)

    for pts in candidates:
        for u, v in pts:
            cv2.circle(vis, (int(u), int(v)), 2, (90, 90, 90), -1)  # cinza: candidatos não usados

    def draw_lane(pts_uv, color):
        if pts_uv is None or len(pts_uv) == 0:
            return
        pts_int = pts_uv.astype(int)
        cv2.polylines(vis, [pts_int.reshape(-1, 1, 2)], False, color, 3)
        for u, v in pts_int:
            cv2.circle(vis, (u, v), 5, color, -1)

    draw_lane(left_uv, (255, 120, 0))    # azul: faixa imediatamente à esquerda
    draw_lane(right_uv, (0, 140, 255))   # laranja: faixa imediatamente à direita

    cx = vis.shape[1] // 2
    cv2.line(vis, (cx, 0), (cx, vis.shape[0]), (0, 255, 0), 1)          # eixo x (y=0, referencial)
    cv2.circle(vis, (cx, vis.shape[0] - 1), 6, (0, 255, 0), -1)          # posição da câmera (0,0)
    return vis


# ======================================================================
# 9. PIPELINE PRINCIPAL
# ======================================================================
def process_frame(bgr_image: np.ndarray, cfg: LaneConfig, tracker: Optional[LaneTracker] = None,
                  return_uv: bool = False):
    """Executa o pipeline completo em um quadro e retorna:
       (imagem_debug_bird's_eye, pontos_faixa_esquerda, pontos_faixa_direita)
    onde os pontos são listas de tuplas (x, y) em metros, no referencial da
    câmera (x para frente, y para a direita, (0,0) na câmera)."""
    # 1) Isola as faixas na perspectiva original
    binary = binarize(bgr_image, cfg)

    # 2) Converte a MÁSCARA para bird's eye view
    M, Minv, src, dst = build_perspective_transform(bgr_image.shape, cfg)
    binary_warped = warp_to_birdseye(binary, M, cfg)
    _, binary_warped = cv2.threshold(binary_warped, 63, 255, cv2.THRESH_BINARY) # Original: 127 a 255

    gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
    gray_warped = warp_to_birdseye(gray_image, M, cfg)
    birdseye_view = cv2.cvtColor(gray_warped, cv2.COLOR_GRAY2BGR)
    birdseye_view[binary_warped > 0] = (0, 0, 255)
    

    #f"{output_stem}_trapezoid.png"

    # 3) Filtro de continuidade espacial
    binary_warped = spatial_continuity_filter(binary_warped, cfg)

    # 4) Extração das faixas candidatas (robusta a curvas, inclusive 90°)
    candidates = find_lane_candidates(binary_warped, cfg)

    # 5) Seleção da faixa imediata à esquerda / à direita
    xm_per_pix, ym_per_pix = compute_scale_factors(cfg, dst)
    left_uv, right_uv = select_immediate_lanes(
        candidates, cfg, xm_per_pix, ym_per_pix, cfg.warped_width, cfg.warped_height
    )

    # 6) Conversão para o referencial da câmera
    left_xy = candidate_to_camera_points(left_uv, cfg, xm_per_pix, ym_per_pix,
                                          cfg.warped_width, cfg.warped_height, cfg.n_output_points)
    right_xy = candidate_to_camera_points(right_uv, cfg, xm_per_pix, ym_per_pix,
                                           cfg.warped_width, cfg.warped_height, cfg.n_output_points)

    # 7) Filtro de continuidade temporal (opcional, para vídeo)
    if tracker is not None:
        left_xy_arr, right_xy_arr = tracker.update(left_xy, right_xy)
        left_xy = [] if left_xy_arr is None else [(float(x), float(y)) for x, y in left_xy_arr]
        right_xy = [] if right_xy_arr is None else [(float(x), float(y)) for x, y in right_xy_arr]

    debug_img = draw_debug_image(binary_warped, candidates, left_uv, right_uv)
    if return_uv:
        overlay_img = overlay_on_original(bgr_image, left_uv, right_uv, Minv, cfg)
        return debug_img, left_xy, right_xy, left_uv, right_uv, binary, birdseye_view, overlay_img
    return debug_img, left_xy, right_xy, binary, birdseye_view


def overlay_on_original(bgr_image, left_uv, right_uv, Minv, cfg):
    """[Bônus] Reprojeta as faixas detectadas de volta na imagem original,
    útil para inspeção visual/depuração (não é a saída pedida, mas ajuda a
    validar a calibração do trapézio de origem)."""
    overlay = np.zeros((cfg.warped_height, cfg.warped_width, 3), dtype=np.uint8)

    def draw(pts_uv, color):
        if pts_uv is None or len(pts_uv) < 2:
            return
        cv2.polylines(overlay, [pts_uv.astype(int).reshape(-1, 1, 2)], False, color, 8)

    draw(left_uv, (255, 120, 0))
    draw(right_uv, (0, 140, 255))
    unwarped = cv2.warpPerspective(overlay, Minv, (bgr_image.shape[1], bgr_image.shape[0]))
    mask = np.any(unwarped > 0, axis=2)
    result = bgr_image.copy()
    result[mask] = cv2.addWeighted(bgr_image, 0.3, unwarped, 0.7, 0)[mask]
    return result


# ======================================================================
# 10. CENA SINTÉTICA DE DEMONSTRAÇÃO / AUTOTESTE
# ======================================================================
def generate_synthetic_frame(cfg: LaneConfig, input_size=(720, 1280)) -> np.ndarray:
    """Gera uma imagem sintética 'como se fosse' capturada pela câmera, a
    partir de uma pista desenhada em vista de topo com uma curva de até 90°
    (reta -> curva fechada -> reta lateral) mais uma faixa tracejada e um
    pouco de ruído (sombra + blob). Serve só para demonstrar/validar o
    pipeline de ponta a ponta sem precisar de um dataset real: a imagem é
    gerada em vista de pássaro e depois deswarpada com Minv (o inverso da
    própria transformação de perspectiva do pipeline), então o pipeline
    completo deve conseguir reconstruir aproximadamente a pista original."""
    h_img, w_img = input_size
    top = np.full((cfg.warped_height, cfg.warped_width, 3), (60, 60, 60), dtype=np.uint8)

    lane_width_px = cfg.warped_width * (1 - 2 * cfg.dst_margin_frac)
    half_lane = lane_width_px / 2.0
    center_x0 = cfg.warped_width / 2.0

    straight_len = cfg.warped_height * 0.55
    R = cfg.warped_height * 0.55
    extra_len = cfg.warped_height * 0.35
    total_len = straight_len + R * (np.pi / 2) + extra_len

    s_vals = np.arange(0, total_len, 1.0)
    positions = np.zeros((len(s_vals), 2))
    normals = np.zeros((len(s_vals), 2))

    p1 = np.array([center_x0, (cfg.warped_height - 1) - straight_len])
    curve_end_s = straight_len + R * (np.pi / 2)
    p_curve_end = p1 + R * np.array([1 - np.cos(np.pi / 2), -np.sin(np.pi / 2)])

    for i, s in enumerate(s_vals):
        if s <= straight_len:
            positions[i] = [center_x0, (cfg.warped_height - 1) - s]
            normals[i] = [1.0, 0.0]
        elif s <= curve_end_s:
            theta = (s - straight_len) / R
            positions[i] = p1 + R * np.array([1 - np.cos(theta), -np.sin(theta)])
            normals[i] = [np.cos(theta), np.sin(theta)]
        else:
            t = s - curve_end_s
            positions[i] = p_curve_end + t * np.array([1.0, 0.0])
            normals[i] = [0.0, 1.0]

    left_pts = positions - normals * half_lane
    right_pts = positions + normals * half_lane

    # Faixa esquerda: sólida
    cv2.polylines(top, [left_pts.astype(int).reshape(-1, 1, 2)], False, (230, 230, 230), 6)

    # Faixa direita: tracejada (testa o fechamento de falhas do filtro de continuidade)
    dash_on, dash_off = 22, 16
    i = 0
    while i < len(right_pts):
        j = min(i + dash_on, len(right_pts))
        seg = right_pts[i:j].astype(int).reshape(-1, 1, 2)
        if len(seg) >= 2:
            cv2.polylines(top, [seg], False, (230, 230, 230), 6)
        i = j + dash_off

    # Ruído: sombra semi-transparente + um pequeno "blob" distrator
    shadow = top.copy()
    cv2.ellipse(shadow, (int(center_x0 - 60), int(cfg.warped_height * 0.35)),
                (90, 160), 20, 0, 360, (25, 25, 25), -1)
    top = cv2.addWeighted(top, 0.6, shadow, 0.4, 0)
    cv2.circle(top, (int(center_x0 + 140), int(cfg.warped_height * 0.75)), 6, (200, 200, 200), -1)

    rng = np.random.default_rng(0)
    noise = rng.normal(0, 4, top.shape).astype(np.int16)
    top = np.clip(top.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    M, Minv, _, _ = build_perspective_transform((h_img, w_img, 3), cfg)
    camera_img = cv2.warpPerspective(top, Minv, (w_img, h_img), flags=cv2.INTER_LINEAR,
                                      borderValue=(40, 40, 40))
    return camera_img


# ======================================================================
# 11. EXECUÇÃO / DEMONSTRAÇÃO
# ======================================================================
def main():
    import argparse
    from pathlib import Path
    global debug

    parser = argparse.ArgumentParser()
    parser.add_argument("input", nargs="?", help="caminho para a imagem de entrada")
    parser.add_argument(
        "--plot-trapezoid",
        action="store_true",
        help="salva a imagem de entrada com o trapezio de conversao desenhado",
    )
    parser.add_argument(
            "--debug",
            action="store_true",
            help="salva imagens para debug e printa pontos obtidos",
        )
    args = parser.parse_args()

    cfg = LaneConfig()
    cfg.show_debug_plots = args.debug

    if args.input:
        input_path = Path(args.input)
        frame = cv2.imread(str(input_path))
        if frame is None:
            raise FileNotFoundError(f"Não consegui abrir a imagem: {args.input}")
        output_stem = input_path.stem
    else:
        print("[info] Nenhuma imagem fornecida como argumento — gerando cena sintética "
              "de demonstração (reta + curva de 90°) para validar o pipeline.")
        print("       Uso real: python lane_segmentation.py caminho/para/imagem.jpg\n")
        frame = generate_synthetic_frame(cfg)
        cv2.imwrite("synthetic_camera_input.png", frame)
        output_stem = "synthetic_camera_input"

    if args.plot_trapezoid:
        _, _, src, _ = build_perspective_transform(frame.shape, cfg)
        trapezoid_img = frame.copy()
        src_int = np.round(src).astype(np.int32)
        cv2.polylines(trapezoid_img, [src_int], isClosed=True, color=(0, 0, 255), thickness=3)
        for point in src_int:
            cv2.circle(trapezoid_img, tuple(point), 7, (0, 255, 255), -1)

        trapezoid_path = f"{output_stem}_trapezoid.png"
        if not cv2.imwrite(trapezoid_path, trapezoid_img):
            raise IOError(f"Não consegui salvar a imagem: {trapezoid_path}")
        print(f"[ok] Trapézio de conversão desenhado e salvo em: {trapezoid_path}")

    if args.debug:
        debug = True

    debug_img, left_xy, right_xy, left_uv, right_uv, binary, birdseye_view, overlay_img = process_frame(frame, cfg, return_uv=True)

    overlay_path = f"{output_stem}_overlay.png"
    cv2.imwrite(overlay_path, overlay_img)

    if debug:
        cv2.imwrite(f"{output_stem}_bev_int.png", birdseye_view)
        
        mask_visual = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask_visual = cv2.cvtColor(mask_visual, cv2.COLOR_GRAY2BGR)
        mask_visual[binary > 0] = (0, 0, 255)
        mask_path = f"{output_stem}_mascara.png"
        cv2.imwrite(mask_path, mask_visual)

        bev_path = f"{output_stem}_bev.png"
        cv2.imwrite(bev_path, debug_img)
        print("[ok] Imagem de saída (bird's eye view com faixas destacadas) salva em: "
                f"{bev_path}")

        print(f"\nFaixa imediatamente à ESQUERDA ({len(left_xy)} pontos) — (x_frente_m, y_direita_m):")
        for x, y in left_xy:
            print(f"   ({x:6.2f}, {y:6.2f})")

        print(f"\nFaixa imediatamente à DIREITA ({len(right_xy)} pontos) — (x_frente_m, y_direita_m):")
        for x, y in right_xy:
            print(f"   ({x:6.2f}, {y:6.2f})")


if __name__ == "__main__":
    main()
