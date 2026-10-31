# ebva_path_control — seguimento de trajetória

## Objetivo

Seguir waypoints, referências Frenet ou geometria LiDAR com métodos Pure Pursuit, Stanley e variantes de esforço/direção. Preserva referências, modelos e unidades de cada método.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_path_control/](ebva_path_control/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [data/](data/) | Referências/amostras pequenas dos métodos. |
| [resources/](resources/) | Recursos auxiliares. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [waypoint_pp](ebva_path_control/waypoint_pp): Pure Pursuit XY.
- [stanley_frenet](ebva_path_control/stanley_frenet): Stanley com erro lateral tipado.
- [curvature_node.py](ebva_path_control/curvature_node.py): porta da variante por curvatura.
- [data](data): referências separadas por método.

## Interfaces

Entradas são pose/odometria, scan e referência conforme o método. Saídas são `AckermannDriveStamped`, `LongitudinalEffortSteering` ou marcadores. `select.launch.py` escolhe `method`; `waypoint_curvature.launch.py` expõe a variante por curvatura. Os contratos locais abaixo fixam tópicos, frames e validade.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_contracts](../ebva_control_contracts/README.md), [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md).
- **ROS e bibliotecas externas:** `ackermann_msgs`, `diagnostic_msgs`, `ament_index_python`, `geometry_msgs`, `nav_msgs`, `python3-numpy`, `python3-yaml`, `rclpy`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

| Executável | Referência/saída |
|---|---|
| waypoint_pp | Referência XY fechada e pose; AckermannDriveStamped |
| waypoint_curvature | Variante de Pure Pursuit com velocidade por curvatura; AckermannDriveStamped |
| csv_frenet_pp | Referência s/e_y; AckermannDriveStamped |
| stanley_frenet | Folgas laterais estampadas e odometria; AckermannDriveStamped |
| waypoint_control | Carrot, Stanley e PP internos; LongitudinalEffortSteering |
| lidar_frenet_pp | Centralização por medianas do scan; LongitudinalEffortSteering |
| waypoint_preview | Marcadores de diagnóstico; não publica comando |

Ackermann representa velocidade e ângulo geométrico no contrato declarado pelo
método. LongitudinalEffortSteering mantém esforço longitudinal adimensional e
ângulo rad, sem chamar esforço de m/s. As fontes originais de tempo são preservadas;
entradas inválidas/antigas não geram uma nova validade de comando.

Os controladores são selecionados antes do adaptador/autoridade/backend do perfil.
Não há conversão universal de suas grandezas para erpm/servo, e testes sintéticos
não fornecem calibração física. Frames, ponto do modelo, referência e sentido
precisam ser declarados na composição.

### Variante por curvatura

~~~bash
ros2 launch ebva_path_control waypoint_curvature.launch.py
ros2 launch ebva_path_control waypoint_curvature.launch.py params_file:=/caminho/curvatura.yaml
~~~

Recebe PoseWithCovarianceStamped em state/pose e publica
control/autonomy/ackermann. expected_frame (map por padrão) e source_timeout_s
validam a origem; stamps repetidos/reordenados ou expirados não são republicados.
O header de saída é o da pose consumida. reference_file seleciona o YAML da
referência; caminhos relativos do CSV são resolvidos a partir desse YAML.

O default usa data/waypoint_curvature/config_example_map.yaml e seu CSV. Foram
preservadas a seleção de alvo, reacquisição e média de curvatura próprias dessa
variante. lookahead_distance=0.5, velocity_gain=0.5,
wheelbase=0.3147253436255231 e curvature_window=5 conservam os valores do método.
A biblioteca não depende de Gym ou de um renderer.

### Contratos dos demais métodos

- [Waypoint PP](docs/waypoint-pp-contract.md)
- [CSV/Frenet](docs/csv-frenet-contract.md)
- [Stanley/Frenet](docs/stanley-frenet-contract.md)
- [Esforço longitudinal e direção](docs/effort-steering-contract.md)
- [Métodos e recursos](docs/path-method-contracts.md)

waypoint_control mantém métodos distintos e esforço longitudinal 0.2;
lidar_frenet_pp mantém esforço 0.4. Stanley/Frenet conserva seu modelo próprio
(incluindo o alvo assinado de velocidade), distinto do Stanley interno de
waypoint_control. Esses parâmetros não demonstram tracking ou calibração em ré.

O CSV time/e_y/y_center_Ld não é uma referência s/e_y. Fixtures sintéticas
acompanham os testes separadamente e não substituem medidas do veículo.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_path_control
source install/local_setup.bash
colcon test --packages-select ebva_path_control --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
