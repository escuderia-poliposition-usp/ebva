# ebva_lane_hough — biblioteca de percepção Hough

## Objetivo

Oferecer percepção alternativa sobre arrays BGR: Canny/Hough, gradiente/Hough com memória, EMA e segmentação rastreada. É biblioteca Python, sem executável ROS instalado ou câmera aberta ao importar.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_lane_hough/](ebva_lane_hough/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [hough_canny.py](ebva_lane_hough/hough_canny.py): Canny/Hough.
- [hough_gradient.py](ebva_lane_hough/hough_gradient.py): gradiente/Hough com memória.
- [ema_filter.py](ebva_lane_hough/ema_filter.py): suavização EMA.
- [tracked_segmentation.py](ebva_lane_hough/tracked_segmentation.py): segmentação rastreada.

## Interfaces

`hough_canny.LaneDetector.process` retorna quatro itens; `hough_gradient.LaneDetector.process` acrescenta confiança. `ema_filter.EMAFilter` fornece `update/reset`; `tracked_segmentation` expõe `LaneConfig` e `LaneTracker`. Os erros têm geometria pixel/normalizada própria, sem adaptação métrica automática.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `python3-numpy`, `python3-opencv`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

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

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_lane_hough
source install/local_setup.bash
colcon test --packages-select ebva_lane_hough --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
