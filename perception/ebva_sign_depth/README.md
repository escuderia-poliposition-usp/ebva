# ebva_sign_depth — segmentação clássica e profundidade

## Objetivo

Segmentar placas circulares vermelhas e comparar profundidade média com estimativa por diâmetro/focal, para inspeção geométrica RGB/depth.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_sign_depth/](ebva_sign_depth/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [data/](data/) | Referências/amostras pequenas dos métodos. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [depth_node.py](ebva_sign_depth/depth_node.py): sincronização, segmentação e estimativas de profundidade.

## Interfaces

`topico_entrada`/`topico_profundidade` recebem `Image` por sincronização aproximada; `topico_camera_info` recebe `CameraInfo`. Saídas `~/debug_image` e `~/debug_image_apenas_placas` são `Image`; estimativas também aparecem em logs. Não publica distância estruturada.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `cv_bridge`, `message_filters`, `python3-numpy`, `python3-opencv`, `rclpy`, `sensor_msgs`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

~~~bash
ros2 run ebva_sign_depth sign_depth
~~~

RGB/profundidade registrada devem estar alinhadas e com mesma resolução.
Parâmetros incluem topico_entrada, topico_profundidade, topico_camera_info,
diametro_real_m, slop_sincronizacao, limiares e publicar_debug.

Profundidade é Z da câmera: 32FC1 em metros ou 16UC1 em milímetros convertido
para metros. Encodings desconhecidos ou dimensões incompatíveis são rejeitados.
Diâmetro e intrínsecos devem corresponder ao objeto e câmera reais.

~~~bash
colcon build --packages-up-to ebva_sign_depth --symlink-install
colcon test --packages-select ebva_sign_depth
colcon test-result --verbose
~~~

Fixtures verificam geometria, unidades e segmentação. Precisão nas imagens
do veículo exige dados e avaliação próprios.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_sign_depth
source install/local_setup.bash
colcon test --packages-select ebva_sign_depth --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
