# ebva_pointcloud_fusion

Projeta LaserScan em PointCloud2, combina a projeção com uma nuvem registrada e
acumula opcionalmente um mapa plano. Os métodos de projeção, transformação,
recorte e voxelização são preservados.

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
