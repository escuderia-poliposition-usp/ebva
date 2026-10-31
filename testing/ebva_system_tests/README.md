# ebva_system_tests — campanhas de software e replays

## Objetivo

Exercitar contratos entre pacotes por campanhas determinísticas e replays, mantendo validação de integração fora dos componentes de operação.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_system_tests/](ebva_system_tests/) | Módulos Python, algoritmos e adaptadores da função. |
| [profiles/](profiles/) | Composições YAML instaladas. |
| [test/](test/) | Verificações e fixtures locais. |
| [data/](data/) | Referências/amostras pequenas dos métodos. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [campaigns.py](ebva_system_tests/campaigns.py): seleção e execução das 16 campanhas.
- [software_replay.py](ebva_system_tests/software_replay.py): infraestrutura de replay.
- [bench_probe.py](ebva_system_tests/bench_probe.py): probe separado, com possibilidade de comandos reais.

## Interfaces

`run_all --output DESTINO` executa 16 campanhas; `--campaign NOME` é repetível. `profiles/*.yaml` contém composições sintéticas. `results.json`, logs e traces registram evidências. `bench_probe` é separado, pode publicar comandos reais e não integra `run_all`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_bringup](../../bringup/ebva_bringup/README.md), [ebva_vehicle_control](../../control/ebva_vehicle_control/README.md), [ebva_path_control](../../control/ebva_path_control/README.md), [ebva_cbf_control](../../control/ebva_cbf_control/README.md), [ebva_frm_control](../../control/ebva_frm_control/README.md), [ebva_mapless_control](../../control/ebva_mapless_control/README.md), [ebva_lane_keeping](../../control/ebva_lane_keeping/README.md), [ebva_localization](../../localization/ebva_localization/README.md), [ebva_lane_detection](../../perception/ebva_lane_detection/README.md), [ebva_lane_hough](../../perception/ebva_lane_hough/README.md), [ebva_recording](../../recording/ebva_recording/README.md), [ebva_simulation](../../simulation/ebva_simulation/README.md), [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md), [ebva_supervisor](../../control/ebva_supervisor/README.md).
- **ROS e bibliotecas externas:** `rosbag2_py`, `ros2bag`, `rosbag2_interfaces`, `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `ackermann_msgs`, `diagnostic_msgs`, `std_msgs`, `std_srvs`, `cv_bridge`, `python3-yaml`, `python3-numpy`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

Os pacotes de operação não dependem deste pacote. As campanhas verificam composições sintéticas; probes físicos têm escopo separado.

## Uso e configuração

~~~bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
ros2 run ebva_system_tests run_all --output /tmp/ebva-campaigns-new
~~~
O destino deve ser novo. --campaign manual seleciona uma campanha. results.json
registra cada comando/retorno. Não executar dois players com serviços fixos no
mesmo domínio. run_all nunca executa bench_probe, Hough com câmera, sensores,
drivers ou stacks SLAM/Nav2. Probes físicos permanecem explícitos e dependem de
escopo próprio. Os replays não permanecem instalados nos pacotes antigos.

Testes unitários dos algoritmos permanecem nos donos. A dependência de testes
para aplicação nunca é invertida. Ferramentas Hough são opcionais e preservam
dependências/pendências de origem, sem qualificação nesta entrega.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_system_tests
source install/local_setup.bash
colcon test --packages-select ebva_system_tests --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
