# Copyright 2026 EBVA (Escuderia Brasileira de Veículos Autônomos)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Segmente placas circulares por borda vermelha e interior branco ou preto.

Estratégia:
1. Converter para HSV e segmentar a cor vermelha (a borda da placa).
   Como o vermelho "quebra" o círculo de matiz (Hue) em 0/180, usamos
   duas faixas de vermelho e somamos as máscaras.
2. Aplicar operações morfológicas para fechar buracos e remover ruído.
3. Preencher o interior do contorno vermelho (a borda é um anel, não um
   disco cheio) para obter a região completa da placa.
4. Encontrar contornos e filtrar por circularidade e área, já que as
   placas são circulares.
5. Filtrar pela cor do MIOLO (interior, sem a borda vermelha): objetos
   vermelhos e redondos que não sejam placas (frutas, bolas, luzes) são
   descartados se o interior não for predominantemente branco/preto.
6. Retornar/salvar a máscara binária (0 = fundo, 255 = placa).

Uso:
    python segmentar_placa.py caminho_da_imagem.jpg
"""

import os
import sys
import cv2
import numpy as np

# Parâmetros filtro cor vermelha
H_min = 170
H_max = 10
S_min = 70
V_min = 50


def segmentar_placa(
    imagem_bgr, area_min=300, circularidade_min=0.6,
    fracao_branco_preto_min=0.6,
):
    """
    Segmente uma imagem BGR em máscaras, contornos e diâmetros de placas.

    Retorna:
        mascara_final : imagem binária (uint8, 0/255) do mesmo tamanho
                         da imagem de entrada, com as regiões de placas
                         marcadas como 255.
        contornos_validos : lista de contornos aceitos (para desenhar
                             caixas/círculos se desejado).
        diametros_px : diâmetro estimado (em pixels) de cada placa.
    """
    # --- 1. Segmentação da cor vermelha em HSV ---
    hsv = cv2.cvtColor(imagem_bgr, cv2.COLOR_BGR2HSV)

    # Faixa 1: vermelho "baixo" (próximo de 0°)
    baixo1 = np.array([0, S_min, V_min])
    alto1 = np.array([H_max, 255, 255])
    mascara1 = cv2.inRange(hsv, baixo1, alto1)

    # Faixa 2: vermelho "alto" (próximo de 180°)
    baixo2 = np.array([H_min, S_min, V_min])
    alto2 = np.array([180, 255, 255])
    mascara2 = cv2.inRange(hsv, baixo2, alto2)

    mascara_vermelho_1 = cv2.bitwise_or(mascara1, mascara2)

    # --- 2. Limpeza morfológica ---
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mascara_vermelho = cv2.morphologyEx(
        mascara_vermelho_1, cv2.MORPH_CLOSE, kernel, iterations=3)
    mascara_vermelho = cv2.morphologyEx(
        mascara_vermelho, cv2.MORPH_OPEN, kernel, iterations=1)

    # --- 3. Preencher o interior do anel vermelho ---
    contornos, hierarquia = cv2.findContours(
        mascara_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    mascara_final = np.zeros(mascara_vermelho.shape, dtype=np.uint8)
    contornos_validos = []
    diametros_px = []

    for cnt in contornos:
        area = cv2.contourArea(cnt)
        if area < area_min:
            continue

        perimetro = cv2.arcLength(cnt, True)
        if perimetro == 0:
            continue

        # Circularidade: 1.0 = círculo perfeito
        circularidade = 4 * np.pi * area / (perimetro ** 2)
        if circularidade < circularidade_min:
            continue

        # Filtro de cor do miolo: descarta objetos vermelhos e redondos
        # que não sejam placas (o miolo de uma placa real é
        # predominantemente branco/preto, não colorido)
        if not miolo_e_branco_ou_preto(
            imagem_bgr, cnt, area, fracao_branco_preto_min
        ):
            continue

        # Preenche o contorno (disco cheio, incluindo miolo branco/preto)
        cv2.drawContours(mascara_final, [cnt], -1, 255, thickness=cv2.FILLED)
        contornos_validos.append(cnt)
        diametros_px.append(estimar_diametro(cnt))

    return mascara_final, contornos_validos, diametros_px, mascara_vermelho_1


def miolo_e_branco_ou_preto(
    imagem_bgr, contorno, area, fracao_min=0.6, margem_erosao=0.25,
):
    """
    Verifique se o interior do contorno é predominantemente branco ou preto.

    A borda vermelha é excluída da medição.

    Passos:
    1. Desenha o disco cheio do contorno em uma máscara.
    2. Erode essa máscara para "comer" a borda vermelha, sobrando só o
       miolo (proporcional ao raio estimado do círculo).
    3. Em HSV, classifica cada pixel do miolo como:
         branco -> baixa saturação e alto valor (brilho)
         preto  -> baixo valor (brilho), independente da saturação
    4. Aceita o contorno se a fração de pixels válidos (branco + preto)
       no miolo for >= fracao_min.

    Isso rejeita objetos vermelhos/redondos (frutas, bolas, luzes, etc.)
    cujo interior não seja branco/preto como o de uma placa real.
    """
    mascara_disco = np.zeros(imagem_bgr.shape[:2], dtype=np.uint8)
    cv2.drawContours(mascara_disco, [contorno], -1, 255, thickness=cv2.FILLED)

    raio_estimado = max(1, int(np.sqrt(area / np.pi)))
    erosao = max(2, int(raio_estimado * margem_erosao))
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (erosao * 2 + 1, erosao * 2 + 1))
    mascara_miolo = cv2.erode(mascara_disco, kernel)

    total_miolo = cv2.countNonZero(mascara_miolo)
    if total_miolo == 0:
        return False

    hsv = cv2.cvtColor(imagem_bgr, cv2.COLOR_BGR2HSV)
    saturacao = hsv[:, :, 1]
    valor = hsv[:, :, 2]

    e_branco = (saturacao < 60) & (valor > 150)
    e_preto = valor < 80
    e_valido = e_branco | e_preto

    pixels_miolo = mascara_miolo == 255
    validos_no_miolo = np.count_nonzero(e_valido & pixels_miolo)

    fracao = validos_no_miolo / total_miolo
    return fracao >= fracao_min


def estimar_diametro(contorno):
    """
    Estime o diâmetro da placa pela altura do seu contorno.

    A medida usa a altura, em pixels, do bounding box.

    Por que altura e não largura?
    - Postes, outras placas, galhos etc. tendem a se sobrepor às laterais
      da placa (eixo x) com mais frequência do que ao topo/base, então a
      largura do contorno é mais suscetível a oclusões parciais.
    - Fotos de rua costumam ter leve inclinação vertical (pitch) e pouca
      rotação de rolagem (roll); isso tende a comprimir a largura aparente
      do círculo com mais frequência do que a altura.
    Por isso a altura do bounding box costuma ser uma estimativa mais
    estável do diâmetro real do círculo do que a largura.
    """
    _, _, largura, altura = cv2.boundingRect(contorno)
    return altura


def principal():
    if len(sys.argv) < 2:
        print("Uso: python segmentar_placa.py <caminho_da_imagem>")
        sys.exit(1)

    caminho = sys.argv[1]
    imagem = cv2.imread(caminho)

    if imagem is None:
        print(f"Não foi possível abrir a imagem: {caminho}")
        sys.exit(1)

    mascara, contornos, diametros_px, mascara_vermelho_1 = segmentar_placa(imagem)

    # Nomes de saída derivados do arquivo de entrada, ex.:
    # "foto.jpg" -> "foto_mascara.png" e "foto_overlay.png"
    pasta = os.path.dirname(caminho)
    nome_base = os.path.splitext(os.path.basename(caminho))[0]

    saida_mascara_vermelha = os.path.join(pasta, f"{nome_base}_mascara_vermelha.png")
    cv2.imwrite(saida_mascara_vermelha, mascara_vermelho_1)

    saida_mascara = os.path.join(pasta, f"{nome_base}_mascara.png")
    cv2.imwrite(saida_mascara, mascara)
    print(f"Máscara salva em: {saida_mascara}")
    print(f"Número de placas detectadas: {len(contornos)}")
    for i, diametro in enumerate(diametros_px, start=1):
        print(f"  Placa {i}: diâmetro estimado = {diametro} px")

    # Visualização opcional: sobrepõe o contorno detectado na imagem original,
    # junto com o diâmetro estimado de cada placa
    imagem_overlay = imagem.copy()
    cv2.drawContours(imagem_overlay, contornos, -1, (0, 255, 0), 2)
    for cnt, diametro in zip(contornos, diametros_px):
        x, y, largura, altura = cv2.boundingRect(cnt)
        texto = f"{diametro}px"
        cv2.putText(imagem_overlay, texto, (x, max(y - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    saida_overlay = os.path.join(pasta, f"{nome_base}_overlay.png")
    cv2.imwrite(saida_overlay, imagem_overlay)
    print(f"Overlay salvo em: {saida_overlay}")


if __name__ == "__main__":
    principal()
