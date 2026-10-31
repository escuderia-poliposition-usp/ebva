# ebva_reference_tools — referências de trajetória

## Objetivo

Coletar, transformar e publicar referências, mantendo separados centro local, folga lateral e pontos globais. A rota bag/JSON permite gerar referência offline.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_reference_tools/](ebva_reference_tools/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [clearance_logger.py](ebva_reference_tools/clearance_logger.py): folgas laterais.
- [extract_path.py](ebva_reference_tools/extract_path.py): extração offline.
- [publish_path.py](ebva_reference_tools/publish_path.py): publicação de referência estática.

## Interfaces

Loggers usam scan/odometria/TF e produzem CSV; centro/folga também em `Float32`. `extract_path` converte bag `PoseStamped` para JSON x/y/yaw. `publish_path` publica `/reference_path` (`nav_msgs/Path`, transient_local). Formatos XY, s/e_y e time/e_y exigem consumidores próprios.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `python3-numpy`, `python3-sklearn`, `rosbag2_py`, `python3-scipy`, `tf2_ros`.
- **Build:** `ament_python`. **Testes:** `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

SciPy fornece suavização gaussiana; a implementação mantém média móvel como alternativa. CSV/JSON e bag precisam ter esquema e frame reconhecidos pelo consumidor.

## Uso e configuração

| Executável | Função |
|---|---|
| centerline_logger | Centro local por geometria LiDAR e erro no lookahead do método |
| clearance_logger | Folgas esquerda/direita e meia diferença lateral |
| reference_points_logger | Pontos de referência em coordenadas da pose |
| trajectory_generator | Caminho a partir de posições do simulador |
| waypoint_collector | Waypoints de TF associados à volta do AutoDRIVE |
| extract_path | Conversão offline de PoseStamped em bag para JSON |
| publish_path | Publicação de referência JSON estática como nav_msgs/Path |

### Coleta

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

### Referência offline

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

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_reference_tools
source install/local_setup.bash
colcon test --packages-select ebva_reference_tools --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
