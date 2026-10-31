# ebva_frm_control — controle reativo por distâncias

## Objetivo

Calcular comandos por diferenças de distâncias LiDAR, preservando os métodos de amostra pontual e média de raios. As saídas têm semânticas distintas e não são velocidades físicas.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_frm_control/](ebva_frm_control/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [point_sample.py](ebva_frm_control/point_sample.py): porta pontual estampada.
- [scan_average.py](ebva_frm_control/scan_average.py): método de média de raios.

## Interfaces

`point_sample`: `sensors/scan` (`LaserScan`) → `control/autonomy/effort` (`DriveEffort`, contrato `frm_point_sample_v1`) e `control/frm_status` (`DiagnosticArray`). `scan_average`: `/scan` → dois `Float32` AutoDRIVE. `select.launch.py` seleciona `method` sem adicionar sensores/adaptadores.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md).
- **ROS e bibliotecas externas:** `python3-numpy`, `rclpy`, `sensor_msgs`, `std_msgs`, `diagnostic_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

Escolha um único método por composição. A porta `point_sample` exige configuração explícita, incluindo frame, como nos requisitos de [seu contrato](docs/point-sample-contract.md). O seletor simples não preenche esse requisito:

~~~bash
ros2 run ebva_frm_control point_sample --ros-args --params-file /dados/frm.yaml
~~~

`scan_average` aceita `ros2 launch ebva_frm_control select.launch.py method:=scan_average`. Os índices 180/360/540/720/900 pressupõem 1080 raios; amostra pontual e média de dez raios mantêm leis/limites diferentes. A saída pontual é esforço adimensional. A campanha `frm` liga essa porta a adaptador específico, autoridade e backend sintéticos; não qualifica atuação física.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_frm_control
source install/local_setup.bash
colcon test --packages-select ebva_frm_control --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
