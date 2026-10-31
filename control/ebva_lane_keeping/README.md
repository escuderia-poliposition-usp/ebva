# ebva_lane_keeping — controle por faixas

## Objetivo

Seguir faixas da percepção e calcular velocidade/ângulo Ackermann. Mantém FSM, perdas, reset e watchdog; câmera, adaptação e atuação são componentes separados.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_lane_keeping/](ebva_lane_keeping/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [lane_keeping_node.py](ebva_lane_keeping/lane_keeping_node.py): wrapper ROS e validade.
- [lane_keeping_core.py](ebva_lane_keeping/lane_keeping_core.py): núcleo de controle.
- [confidence_fsm.py](ebva_lane_keeping/confidence_fsm.py): estados de confiança/perda.

## Interfaces

`lane_topic` recebe `lane_msgs/LaneBoundaries`; `odom_topic` recebe `nav_msgs/Odometry`. `command_topic` publica `AckermannDriveStamped` (default `/ackermann_cmd`). Debug em `/lane_keeping/debug` (`Float32MultiArray`); reset em `/lane_keeping/reset` é tópico `std_msgs/Empty`, apesar do nome do parâmetro `reset_service`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [lane_msgs](../../interfaces/lane_msgs/README.md), [ebva_control_contracts](../ebva_control_contracts/README.md).
- **ROS e bibliotecas externas:** `ackermann_msgs`, `nav_msgs`, `python3-numpy`, `rclpy`, `std_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

O produtor de faixas é escolhido pela composição. O build depende de lane_msgs, sem exigir câmera ou detector em execução.

## Uso e configuração

~~~bash
ros2 launch ebva_lane_keeping select.launch.py
~~~

### Entrada, geometria e saída

LaneBoundaries usa x para frente e y para a direita no modelo de câmera.
A composição deve declarar expected_lane_frame e a correspondência geométrica;
não presuma FLU por causa do nome de frame.

A saída é Ackermann em m/s e rad. command_frame e command_topic declaram sua
ligação ao consumidor; o default de tópico é /ackermann_cmd. O controlador
não inicia mux ou VESC. Os perfis escolhem detector, método, adaptador e
autoridade conforme seus contratos.

### Validade e ciclo de vida

O wrapper exige stamp original fresco/crescente antes de chamar o núcleo.
Watchdog combina prazo monotônico e idade original, sem renovação por
duplicatas ou relógio de replay parado. Reset limpa o lease; reinicie o
perfil para iniciar outra época de replay.

FSM, perdas de linha, latch, reset, watchdog e fallback de velocidade têm
testes de software. As configurações de fixture não medem câmera, montagem,
escala, centro ou resposta física do veículo.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_lane_keeping
source install/local_setup.bash
colcon test --packages-select ebva_lane_keeping --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

A campanha lane_control_replay pertence a ebva_system_tests e utiliza entradas
sintéticas com adaptador/autoridade/backend. Veja [testes](../../docs/testing.md),
[interfaces](../../docs/interfaces.md) e
[atribuições](../../THIRD_PARTY_NOTICES.md).

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
