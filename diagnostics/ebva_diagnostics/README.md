# ebva_diagnostics — observação de estado

## Objetivo

Observar odometria e deslocamento entre map/odom para diagnóstico. As ferramentas exibem/registram dados, sem novas estimativas ou comandos de atuação.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_diagnostics/](ebva_diagnostics/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [odometry_viewer.py](ebva_diagnostics/odometry_viewer.py): visualização no terminal.
- [map_odom_distance.py](ebva_diagnostics/map_odom_distance.py): observação TF e log.

## Interfaces

`odometry_viewer` recebe `/odometry/pose_source/filtered` (`Odometry`) e exibe Rich. `map_odon_distance` consulta TF map→odom, publica `/map_odom_distance` (`Float32`) e escreve log. O executável contém `odon`; seu módulo é `map_odom_distance.py`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `tf2_ros`, `std_msgs`, `nav_msgs`, `python3-rich`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

### Interfaces

`map_odon_distance` consulta TF map→odom, publica /map_odom_distance e
acrescenta linhas a um arquivo. Defaults: log_directory /tmp e log_filename
map_odom_distance.txt. A escrita faz parte de sua execução.

`odometry_viewer` assina /odometry/pose_source/filtered (nav_msgs/Odometry)
e apresenta valores no terminal com rich.

~~~bash
ros2 run ebva_diagnostics odometry_viewer
~~~

Selecione explicitamente as ferramentas e confira os frames/timestamps
da composição observada.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_diagnostics
source install/local_setup.bash
colcon test --packages-select ebva_diagnostics --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Mantenha diagnósticos fora do Git. Import/build não executam a coleta.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
