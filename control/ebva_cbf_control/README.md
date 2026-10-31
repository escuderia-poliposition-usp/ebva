# ebva_cbf_control — filtros por funções de barreira

## Objetivo

Filtrar comandos nominais com os modelos distintos yaw_rate, yaw_rate_margin e steering_tangent. As portas estampadas identificam modelo, unidades e medidas consumidas.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_cbf_control/](ebva_cbf_control/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [yaw_rate](ebva_cbf_control/yaw_rate): kernel yaw rate.
- [yaw_rate_margin](ebva_cbf_control/yaw_rate_margin): kernel com margem.
- [steering_tangent](ebva_cbf_control/steering_tangent): kernel em tangente de direção.
- [yaw_rate_node.py](ebva_cbf_control/yaw_rate_node.py): porta ROS estampada yaw rate.

## Interfaces

`*_stamped` recebe `LaserScan`, `Odometry` e nominal tipado, publicando `CbfYawRateCommand`, `CbfYawRateMarginCommand` ou `CbfSteeringTangentCommand`. Defaults relativos: `perception/scan`, `state/odometry`, `control/autonomy/nominal`, `control/autonomy/cbf_<modelo>`; status `String` em `~/status`. `*_filter` mantém a interface anterior AutoDRIVE.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md), [ebva_control_contracts](../ebva_control_contracts/README.md).
- **ROS e bibliotecas externas:** `nav_msgs`, `python3-numpy`, `rclpy`, `sensor_msgs`, `std_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

Escolha modelo/contrato antes de ligar nominal, scan e velocidade. As configurações de config/testing e as campanhas CBF são de software. Uma porta estampada recebe YAML explícito:

~~~bash
ros2 run ebva_cbf_control yaw_rate_stamped --ros-args --params-file /dados/cbf.yaml
~~~

`ros2 launch ebva_cbf_control select.launch.py model:=yaw_rate` inicia a interface anterior do filtro. Ela usa `/scan`, `/odom`, `/a_no`, `/w_no` e publica dois Float32 AutoDRIVE. `rc_input` e `vehicle_output` não são argumentos suportados; não há leitor serial ou ponte VESC neste pacote.

As seis entradas instaladas são yaw_rate_stamped, yaw_rate_margin_stamped, steering_tangent_stamped e os três nomes correspondentes terminados em _filter.

Consulte [caracterização](docs/cbf-software-contract.md), [yaw_rate estampado](docs/yaw-rate-ros-contract.md) e [modelos margin/tangent](docs/alternate-model-ros-contracts.md). Steering_tangent filtra `tan(delta)`; modelos não são aliases. As portas estampadas rejeitam entradas inválidas/antigas/inconsistentes ou QP inadmissível. O solver Hildreth e as conversões não certificam segurança física.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_cbf_control
source install/local_setup.bash
colcon test --packages-select ebva_cbf_control --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
