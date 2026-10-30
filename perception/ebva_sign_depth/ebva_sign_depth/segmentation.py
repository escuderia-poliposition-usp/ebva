"""
Segmentação de placas de trânsito circulares (borda vermelha, fundo branco,
símbolos pretos) a partir de uma imagem.

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
7. Classificar cada placa recortada comparando-a (por correlação, após
   normalizar em cinza/tamanho fixo) com 6 templates de referência:
   esq.jpg, dir.jpg, p_esq.jpg, p_dir.jpg, fre.jpg, p_fre.jpg.

Uso:
    python segmentar_placa.py caminho_da_imagem.jpg [pasta_templates]
"""

import os
import sys
import cv2
import numpy as np


def segmentar_placa(imagem_bgr, area_min=300, circularidade_min=0.6,
                     fracao_branco_preto_min=0.6):
    """
    Recebe uma imagem BGR (como lida pelo cv2.imread) e retorna:
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
    baixo1 = np.array([0, 70, 50])
    alto1 = np.array([10, 255, 255])
    mascara1 = cv2.inRange(hsv, baixo1, alto1)

    # Faixa 2: vermelho "alto" (próximo de 180°)
    baixo2 = np.array([170, 70, 50])
    alto2 = np.array([180, 255, 255])
    mascara2 = cv2.inRange(hsv, baixo2, alto2)

    mascara_vermelho = cv2.bitwise_or(mascara1, mascara2)

    # --- 2. Limpeza morfológica ---
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mascara_vermelho = cv2.morphologyEx(mascara_vermelho, cv2.MORPH_CLOSE,
                                         kernel, iterations=3)
    mascara_vermelho = cv2.morphologyEx(mascara_vermelho, cv2.MORPH_OPEN,
                                         kernel, iterations=1)

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
        if not miolo_e_branco_ou_preto(imagem_bgr, cnt, area,
                                        fracao_branco_preto_min):
            continue

        # Preenche o contorno (disco cheio, incluindo miolo branco/preto)
        cv2.drawContours(mascara_final, [cnt], -1, 255, thickness=cv2.FILLED)
        contornos_validos.append(cnt)
        diametros_px.append(estimar_diametro(cnt))

    return mascara_final, contornos_validos, diametros_px


def miolo_e_branco_ou_preto(imagem_bgr, contorno, area, fracao_min=0.6,
                             margem_erosao=0.25):
    """
    Verifica se o INTERIOR de um contorno (excluindo a borda vermelha)
    é predominantemente branco ou preto.

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
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                        (erosao * 2 + 1, erosao * 2 + 1))
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
    Estima o diâmetro (em pixels) de uma placa a partir do seu contorno,
    usando a ALTURA (dimensão em y) do bounding box.

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


def aplicar_mascara_na_imagem(imagem_bgr, mascara):
    """
    Retorna uma imagem do MESMO TAMANHO da entrada, com os pixels fora
    da máscara zerados (pretos) e os pixels dentro da máscara mantidos
    com a cor original.

    mascara: imagem binária (0/255), mesmo tamanho de imagem_bgr
             (ex.: a mascara_final retornada por segmentar_placa).
    """
    return cv2.bitwise_and(imagem_bgr, imagem_bgr, mask=mascara)


def recortar_placas(imagem_bgr, contornos, mascara=None, margem=0):
    """
    Recorta, para cada contorno detectado, a menor região retangular
    (bounding box) que contém a placa — ou seja, o "tamanho necessário".

    mascara: se fornecida (a mascara_final de segmentar_placa), é
             aplicada ANTES do recorte, então os cantos do bounding box
             fora do círculo da placa também ficam pretos. Sem isso, o
             recorte é só um retângulo "cru" da imagem original — os
             cantos fora do círculo mantêm a cor de fundo original.
    margem: pixels extras adicionados em volta do recorte (opcional),
            já limitados às bordas da imagem.

    Retorna uma lista de imagens (arrays numpy), uma por placa, na
    mesma ordem de `contornos`.
    """
    imagem_fonte = imagem_bgr
    if mascara is not None:
        imagem_fonte = aplicar_mascara_na_imagem(imagem_bgr, mascara)

    altura_img, largura_img = imagem_fonte.shape[:2]
    recortes = []

    for cnt in contornos:
        x, y, w, h = cv2.boundingRect(cnt)

        x0 = max(0, x - margem)
        y0 = max(0, y - margem)
        x1 = min(largura_img, x + w + margem)
        y1 = min(altura_img, y + h + margem)

        recortes.append(imagem_fonte[y0:y1, x0:x1])

    return recortes


# --- Classificação das placas por comparação com templates ---
#
# Cada template é uma imagem de referência com fundo preto (mesmo
# formato gerado por aplicar_mascara_na_imagem/recortar_placas), uma
# por classe de placa. O nome do arquivo mapeia para o rótulo da classe.
TEMPLATES_PADRAO = {
    "esq.jpg": "Vire a esquerda",
    "dir.jpg": "Vire a direita",
    "p_esq.jpg": "Proibido virar a esquerda",
    "p_dir.jpg": "Proibido virar a direita",
    "fre.jpg": "Siga em frente",
    "p_fre.jpg": "Proibido seguir em frente",
}


def preparar_para_comparacao(imagem_bgr, tamanho=128):
    """
    Normaliza uma imagem (recorte ou template) para comparação:
    escala de cinza + redimensionamento para um tamanho fixo, já que
    recortes de placas diferentes têm dimensões diferentes.
    """
    cinza = cv2.cvtColor(imagem_bgr, cv2.COLOR_BGR2GRAY)
    return cv2.resize(cinza, (tamanho, tamanho), interpolation=cv2.INTER_AREA)


def carregar_templates(pasta_templates, tamanho=128):
    """
    Carrega e prepara (cinza + redimensiona) cada template listado em
    TEMPLATES_PADRAO a partir de pasta_templates.

    Retorna um dicionário {rotulo: imagem_preparada}. Templates cujo
    arquivo não for encontrado são ignorados (com aviso no console).
    """
    templates_preparados = {}

    for nome_arquivo, rotulo in TEMPLATES_PADRAO.items():
        caminho = os.path.join(pasta_templates, nome_arquivo)
        template = cv2.imread(caminho)

        if template is None:
            print(f"  [aviso] template não encontrado: {caminho}")
            continue

        templates_preparados[rotulo] = preparar_para_comparacao(template, tamanho)

    return templates_preparados


def classificar_placa(recorte_bgr, templates_preparados, tamanho=128):
    """
    Classifica um recorte de placa (fundo preto) comparando-o com cada
    template via correlação normalizada (cv2.matchTemplate,
    TM_CCOEFF_NORMED) sobre as versões em cinza/redimensionadas.

    Retorna (rotulo_mais_provavel, score), onde score vai de -1 a 1
    (quanto maior, mais parecido). Se templates_preparados estiver
    vazio, retorna (None, 0.0).
    """
    if not templates_preparados:
        return None, 0.0

    alvo = preparar_para_comparacao(recorte_bgr, tamanho)

    melhor_rotulo = None
    melhor_score = -1.0

    for rotulo, template in templates_preparados.items():
        resultado = cv2.matchTemplate(alvo, template, cv2.TM_CCOEFF_NORMED)
        score = float(resultado[0][0])
        if score > melhor_score:
            melhor_score = score
            melhor_rotulo = rotulo

    return melhor_rotulo, melhor_score


def principal():
    if len(sys.argv) < 2:
        print("Uso: python segmentar_placa.py <caminho_da_imagem> [pasta_templates]")
        sys.exit(1)

    caminho = sys.argv[1]
    imagem = cv2.imread(caminho)

    if imagem is None:
        print(f"Não foi possível abrir a imagem: {caminho}")
        sys.exit(1)

    # Pasta com esq.jpg, dir.jpg, p_esq.jpg, p_dir.jpg, fre.jpg, p_fre.jpg.
    # Por padrão, procura na mesma pasta deste script; pode ser
    # sobrescrita passando um terceiro argumento na linha de comando.
    if len(sys.argv) >= 3:
        pasta_templates = sys.argv[2]
    else:
        pasta_templates = os.path.dirname(os.path.abspath(__file__))
    templates = carregar_templates(pasta_templates)

    mascara, contornos, diametros_px = segmentar_placa(imagem)

    # Nomes de saída derivados do arquivo de entrada, ex.:
    # "foto.jpg" -> "foto_mascara.png" e "foto_overlay.png"
    pasta = os.path.dirname(caminho)
    nome_base = os.path.splitext(os.path.basename(caminho))[0]

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

    # Imagem nas dimensões originais, com tudo preto exceto as placas
    imagem_apenas_placas = aplicar_mascara_na_imagem(imagem, mascara)
    saida_apenas_placas = os.path.join(pasta, f"{nome_base}_apenas_placas.png")
    cv2.imwrite(saida_apenas_placas, imagem_apenas_placas)
    print(f"Imagem com fundo preto salva em: {saida_apenas_placas}")

    # Recortes individuais, cada um no menor tamanho que contém a placa
    recortes = recortar_placas(imagem, contornos, mascara=mascara)
    for i, recorte in enumerate(recortes, start=1):
        saida_recorte = os.path.join(pasta, f"{nome_base}_placa_{i}.png")
        cv2.imwrite(saida_recorte, recorte)

        rotulo, score = classificar_placa(recorte, templates)
        if rotulo is not None:
            print(f"Recorte da placa {i} salvo em: {saida_recorte} "
                  f"({recorte.shape[1]}x{recorte.shape[0]} px) "
                  f"-> classificada como: {rotulo} (score={score:.3f})")
        else:
            print(f"Recorte da placa {i} salvo em: {saida_recorte} "
                  f"({recorte.shape[1]}x{recorte.shape[0]} px) "
                  f"-> não classificada (nenhum template carregado)")


if __name__ == "__main__":
    principal()
