# ebva_localization

Adaptadores de estado e TF, estimadores IMU e interfaces de localização/mapeamento.
O pacote processa tópicos ROS; não inicia sensores, aquisição RC ou driver VESC.

Interfaces launch: mapping_zed, mapping_imu, mapping_ekf, slam_localization,
amcl, ekf e imu_prior. Os arquivos aceitam params_file quando há configuração de
domínio. O prior legacy_imu_prior conserva um método distinto de imu_odometry SI.

Mapas de localização são entradas explícitas em config/slam/slam_toolbox_localization.yaml
(basename dos arquivos posegraph/data) e config/amcl.yaml
(map_server.yaml_filename). Campos vazios precisam ser preenchidos para a composição
escolhida. Parâmetros/extrínsecos físicos não são estimados por esses launches.

## Pose para gravação

~~~bash
ros2 run ebva_localization pose_recorder --ros-args -p map_frame:=map -p base_frame:=zed_camera_link
~~~

Publica /robot_pose (PoseStamped) a 10 Hz a partir do TF map_frame -> base_frame,
mantendo o stamp do TF usado. A gravação da bag é selecionada separadamente.
odometry_pose, pose_republisher, base_link_pose e odometry_pose_tf são adaptadores
distintos, escolhidos conforme o contrato de estado; não lance publicadores
concorrentes do mesmo TF.

## Persistência de mapa

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

## Solicitação única ao serviço SaveMap

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
