# ebva_lane_detection — percepção de faixas por segmentação

## Objetivo

Extrair faixas por máscara de pista ou cor/gradiente e projetar pontos no plano configurado da câmera, entregando geometria e visualização para inspeção.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_lane_detection/](ebva_lane_detection/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [lane_detection_node.py](ebva_lane_detection/lane_detection_node.py): entrada/saídas ROS.
- [segmentation_methods.py](ebva_lane_detection/segmentation_methods.py): seleção dos métodos.
- [image_contract.py](ebva_lane_detection/image_contract.py): tratamento do contrato de imagem.

## Interfaces

`image_topic` recebe `sensor_msgs/Image` com QoS de sensor. Publica `/lane_detection/lane_boundaries` (`LaneBoundaries`), `/lane_detection/birdseye_debug` (`Image`) e `/lane_detection/status` (`String`). `lane_detection.launch.py` seleciona `params_file`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [lane_msgs](../../interfaces/lane_msgs/README.md).
- **ROS e bibliotecas externas:** `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`, `python3-opencv`, `python3-numpy`, `python3-matplotlib`, `rosbag2_py`, `rosbag2_transport`, `rosbag2_interfaces`, `ros2bag`.
- **Build:** `ament_python`. **Testes:** `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`, `ament_index_python`.

## Uso e configuração

~~~bash
ros2 launch ebva_lane_detection lane_detection.launch.py params_file:=/dados/faixas.yaml
~~~

### Métodos

- road_mask_v4_80: máscara de pista com limiar 80.
- road_mask_v4_50: variante com limiar 50.
- color_gradient_v3: binarização por cor/gradiente.

A seleção é um parâmetro do detector e não muda automaticamente as unidades
da geometria. Os métodos alternativos Hough permanecem na biblioteca
ebva_lane_hough.

### Geometria e tempo

LaneBoundaries contém header e vetores left_lane/right_lane. O modelo desta
implementação usa x para frente, y para a direita, origem na câmera e z zero.
Metros são calculados a partir da homografia/escala configuradas. Isso não
aplica TF nem mede extrínsecos automaticamente.

O header de medida conserva o timestamp da imagem; camera_frame_id identifica
o plano declarado. O debug é um overlay na imagem original e conserva seu
header óptico. Uma faixa vazia indica ausência no quadro; o wrapper não atribui
stamp novo a geometria antiga durante silêncio.

Parâmetros incluem image_topic, camera_frame_id, dimensões físicas assumidas,
ROI/homografia, method e publish_debug_image. Eles são lidos na construção:
reinicie o nó após alteração. Confira o
[contrato dos métodos](docs/lane-method-contract.md).

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_lane_detection
source install/local_setup.bash
colcon test --packages-select ebva_lane_detection --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Fontes sintéticas ficam em ebva_simulation e replays em ebva_system_tests.
Testes verificam máscara, geometria, encodings e headers; avaliação física da
câmera e comparação de qualidade exigem dados com critérios explícitos.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
