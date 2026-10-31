# ebva_pointcloud_fusion — projeção e fusão de nuvens

## Objetivo

Projetar LiDAR em nuvem, fundir com nuvem registrada e acumular opcionalmente mapa plano. O processamento C++ usa TF, filtros e voxelização; os sensores são selecionados separadamente.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [src/](src/) | Implementação C++. |
| [include/](include/) | Headers C++. |
| [package.xml](package.xml), [CMakeLists.txt](CMakeLists.txt) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [fusion_node.cpp](src/fusion_node.cpp): projeção, transformação, fusão e acumulação.
- [fusion.yaml](config/fusion.yaml): frames, filtros e persistência.

## Interfaces

`scan_topic` recebe `LaserScan` (default `/scan`); `cloud_topic` recebe `PointCloud2` quando `use_cloud=true`. Publica `PointCloud2` em `/fused_cloud` e `/fused_map`. TF liga as medidas a `target_frame`/`map_frame`. `fusion.launch.py` recebe `params_file`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclcpp`, `sensor_msgs`, `message_filters`, `laser_geometry`, `tf2_ros`, `tf2_sensor_msgs`, `pcl_conversions`, `std_srvs`, `launch`, `launch_ros`, `ament_index_python`.
- **Build:** `ament_cmake`. **Testes:** `ament_cmake_pytest`, `rclpy`.

O build C++ também localiza PCL pelo CMake. Sensores e TF precisam existir na composição; `use_cloud=false` dispensa nuvem de câmera.

## Uso e configuração

~~~bash
ros2 launch ebva_pointcloud_fusion fusion.launch.py
ros2 launch ebva_pointcloud_fusion fusion.launch.py params_file:=/caminho/fusion.yaml
~~~

O launch inicia apenas processamento. Produtores de scan, nuvem e TF são
selecionados pelo perfil; nenhum sensor ou atuador é iniciado por este pacote.
Entradas: scan_topic (LaserScan) e cloud_topic (PointCloud2, se use_cloud=true).
Saídas: /fused_cloud e /fused_map. use_cloud=false seleciona projeção somente LiDAR.
target_frame é o frame de fusão e map_frame é o frame do mapa acumulado.
Os tópicos podem ser remapeados por ROS.

config/fusion.yaml conserva voxel_leaf_size=0.05 m, map_height_tolerance=0.05 m e
map_publish_rate=2 Hz. map_save_path vazio desabilita persistência; um caminho
explícito habilita a gravação. Cada destino é reservado exclusivamente e recebe
um identificador único, inclusive em gravações no mesmo segundo. Frames e extrínsecos precisam
corresponder à montagem. PCL é dependência C++ do build.

O teste de projeção usa LaserScan sintético e domínio ROS isolado, sem dispositivos.
Não qualifica a geometria ou os extrínsecos do veículo.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_pointcloud_fusion
source install/local_setup.bash
colcon test --packages-select ebva_pointcloud_fusion --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
