# ebva_localization — estado, TF e mapas

## Objetivo

Integrar localização/mapeamento, adaptar pose/odometria e persistir mapas. Reúne SLAM, AMCL, EKF e priors IMU distintos; cada perfil define fontes e autoridade de TF.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_localization/](ebva_localization/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [imu_odometry.py](ebva_localization/imu_odometry.py): prior IMU SI.
- [legacy_imu_prior.py](ebva_localization/legacy_imu_prior.py): método IMU alternativo.
- [slam_map_saver.py](ebva_localization/slam_map_saver.py): persistência composta.
- [state_tf_audit.py](ebva_localization/state_tf_audit.py): observação de autoridades TF.

## Interfaces

Launches `mapping_*`, `slam_localization`, `amcl`, `ekf`, `imu_prior`; adaptadores de `Odometry`/`PoseStamped`/`PoseWithCovarianceStamped` e TF. `state_tf_audit` observa TF; `pose_recorder` publica `/robot_pose` sem gravar bag. `slam_map_saver` e `save_slam_map_once` oferecem persistência com contratos diferentes.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `sensor_msgs`, `tf2_ros`, `tf2_msgs`, `python3-numpy`, `rosbag2_py`, `rosbag2_interfaces`, `ros2bag`, `rosbag2_storage_default_plugins`, `ament_index_python`, `launch`, `launch_ros`, `python3-yaml`, `slam_toolbox`, `robot_localization`, `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`, `tf2_geometry_msgs`, `std_srvs`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

SLAM, AMCL e EKF são alternativas. Mapas de ocupação/serializados, TF e medidas são entradas da composição; os launches não estimam extrínsecos automaticamente.

## Uso e configuração

Interfaces launch: mapping_zed, mapping_imu, mapping_ekf, slam_localization,
amcl, ekf e imu_prior. Os arquivos aceitam params_file quando há configuração de
domínio. O prior legacy_imu_prior conserva um método distinto de imu_odometry SI.

Mapas de localização são entradas explícitas em config/slam/slam_toolbox_localization.yaml
(basename dos arquivos posegraph/data) e config/amcl.yaml
(map_server.yaml_filename). Campos vazios precisam ser preenchidos para a composição
escolhida. Parâmetros/extrínsecos físicos não são estimados por esses launches.

### Pose para gravação

~~~bash
ros2 run ebva_localization pose_recorder --ros-args -p map_frame:=map -p base_frame:=zed_camera_link
~~~

Publica /robot_pose (PoseStamped) a 10 Hz a partir do TF map_frame -> base_frame,
mantendo o stamp do TF usado. A gravação da bag é selecionada separadamente.
odometry_pose, pose_republisher, base_link_pose e odometry_pose_tf são adaptadores
distintos, escolhidos conforme o contrato de estado; não lance publicadores
concorrentes do mesmo TF.

### Persistência de mapa

~~~bash
ros2 run ebva_localization slam_map_saver --ros-args -p map_save_path:=/caminho/saida/mapa
ros2 service call /slam_map_saver/save_map std_srvs/srv/Trigger '{}'
~~~

map_save_path vazio desabilita gravação. O nó observa /map (OccupancyGrid) e
oferece save_map; gera PGM/YAML e PCD, e solicita posegraph/data ao serviço
/slam_toolbox/serialize_map. O prefixo ganha timestamp e identificador temporal
para distinguir salvamentos. Arquivos existentes não são sobrescritos.
O salvamento no encerramento é de melhor esforço; use o serviço antes de parar
o stack para confirmar a disponibilidade do posegraph.

As funções de mapa mantêm os métodos de conversão de células, origem e rotação.
Testes com dados sintéticos não qualificam SLAM, deriva ou precisão física.

### Solicitação única ao serviço SaveMap

~~~bash
ros2 run ebva_localization save_slam_map_once --ros-args -p output_prefix:=/caminho/saida/mapa -p timeout_s:=10.0
~~~

Essa interface solicita diretamente /slam_toolbox/save_map; não captura /map
localmente e não substitui slam_map_saver. output_prefix é obrigatório, em
diretório existente e sem extensão .pgm/.yaml. Os destinos são verificados antes
da requisição; arquivos existentes são recusados. O servidor externo controla
a escrita, portanto não execute outros escritores concorrentes no mesmo prefixo.

O prazo timeout_s é finito e inclui espera pelo serviço e pela resposta.
service_name permite selecionar outra instância compatível. A API Humble usa
request.name.data (std_msgs/String) e response.result: somente RESULT_SUCCESS=0
retorna sucesso. RESULT_NO_MAP_RECEIEVD=1, RESULT_UNDEFINED_FAILURE=255 e timeout
retornam falha. O processo encerra após uma solicitação; códigos de saída são
0 (sucesso), 1 (serviço/falha/timeout), 2 (entrada inválida/destino existente).

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_localization
source install/local_setup.bash
colcon test --packages-select ebva_localization --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
