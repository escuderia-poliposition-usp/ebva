# ebva_lane_detection — faixas por segmentação

O pacote recebe sensor_msgs/Image e publica lane_msgs/LaneBoundaries, imagem
de debug e diagnóstico. O launch lane_detection.launch.py recebe params_file,
usando config/lane_detection_params.yaml por padrão.

~~~bash
ros2 launch ebva_lane_detection lane_detection.launch.py params_file:=/dados/faixas.yaml
~~~

## Métodos

- road_mask_v4_80: máscara de pista com limiar 80.
- road_mask_v4_50: variante com limiar 50.
- color_gradient_v3: binarização por cor/gradiente.

A seleção é um parâmetro do detector e não muda automaticamente as unidades
da geometria. Os métodos alternativos Hough permanecem na biblioteca
ebva_lane_hough.

## Geometria e tempo

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

## Testes

~~~bash
colcon build --packages-up-to ebva_lane_detection --symlink-install
colcon test --packages-select ebva_lane_detection
colcon test-result --verbose
~~~

Fontes sintéticas ficam em ebva_simulation e replays em ebva_system_tests.
Testes verificam máscara, geometria, encodings e headers; avaliação física da
câmera e comparação de qualidade exigem dados com critérios explícitos.
