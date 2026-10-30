# ebva_reference_tools

Coleta e processamento de referências de trajetória. Os métodos de centro local,
folgas laterais e pontos globais permanecem identificados, pois suas saídas não
representam a mesma medição.

| Executável | Função |
|---|---|
| centerline_logger | Centro local por geometria LiDAR e erro no lookahead do método |
| clearance_logger | Folgas esquerda/direita e meia diferença lateral |
| reference_points_logger | Pontos de referência em coordenadas da pose |
| trajectory_generator | Caminho a partir de posições do simulador |
| waypoint_collector | Waypoints de TF associados à volta do AutoDRIVE |
| extract_path | Conversão offline de PoseStamped em bag para JSON |
| publish_path | Publicação de referência JSON estática como nav_msgs/Path |

## Coleta

~~~bash
ros2 run ebva_reference_tools clearance_logger --ros-args -p output_file:=/caminho/folgas.csv
ros2 run ebva_reference_tools waypoint_collector --ros-args -p output_file:=/caminho/waypoints.csv
~~~

centerline_logger, clearance_logger, reference_points_logger e waypoint_collector
exigem output_file em diretório existente. Criação é exclusiva: não sobrescrevem
arquivos. Tópicos podem ser remapeados. Os métodos LiDAR usam /scan e
/odometry/pose_source/filtered por padrão; os coletores originais usam as últimas
amostras disponíveis, sem alegar sincronização ou ground truth. Saídas Float32
e formatos CSV desses métodos precisam de um adaptador explícito para contratos
estampados de controle. Não relabelar meia diferença de folgas como erro cartesiano.

## Referência offline

~~~bash
ros2 run ebva_reference_tools extract_path /caminho/bag /caminho/referencia.json --ds 0.1 --smooth 5
ros2 run ebva_reference_tools publish_path --ros-args -p waypoints_file:=/caminho/referencia.json -p frame_id:=map
~~~

extract_path lê PoseStamped de /robot_pose, reamostra por comprimento de arco e
suaviza pela implementação existente. --topic seleciona outra origem. O JSON
contém x/y/yaw; a saída deve ser nova. SciPy fornece a suavização gaussiana; o
método mantém sua alternativa de média móvel quando usado sem essa dependência.

publish_path valida valores finitos e publica /reference_path com durabilidade
transient_local. O stamp identifica a geração da referência estática, não uma
medida de posição do veículo. A referência não inclui velocidade nem conversão
para comandos de motor.
