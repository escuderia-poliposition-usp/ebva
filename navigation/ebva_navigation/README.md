# ebva_navigation — Nav2 e ponte cinemática

## Objetivo

Configurar Nav2 para o modelo Ackermann e converter Twist para velocidade/ângulo. O perfil completo acrescenta localização, sensores e atuação; a ponte isolada não os inicia.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_navigation/](ebva_navigation/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [cmd_vel_to_ackermann_node.py](ebva_navigation/cmd_vel_to_ackermann_node.py): ponte cinemática.
- [bridge.yaml](config/bridge.yaml): parâmetros da ponte.
- [nav2_params.yaml](config/nav2_params.yaml): configuração Nav2 padrão.

## Interfaces

`navigation.launch.py` inicia Nav2; `bridge.launch.py` inicia `cmd_vel_to_ackermann_node`. `cmd_vel_topic` (`Twist`, default `/cmd_vel`) → `ackermann_cmd_topic` (`AckermannDriveStamped`, default relativo `control/autonomy/ackermann`). Entre-eixos, limite e frame pertencem a `config/bridge.yaml`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `geometry_msgs`, `ackermann_msgs`, `nav2_bringup`, `nav2_amcl`, `nav2_map_server`, `nav2_smac_planner`, `nav2_regulated_pure_pursuit_controller`, `ament_index_python`, `launch`, `launch_ros`, `python3-yaml`.
- **Build:** `ament_python`. **Testes:** `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

## Uso e configuração

- navigation.launch.py inicia a composição Nav2.
- bridge.launch.py converte Twist para Ackermann segundo o modelo configurado.

Ambas aceitam params_file com default do pacote. Elas não iniciam localização,
sensores, RC ou driver VESC. Perfis completos incluem esses componentes
explicitamente.

A ponte publica control/autonomy/ackermann. O adaptador de veículo e a
autoridade/backend ficam em ebva_vehicle_control; não há escrita direta nos
comandos finais nesta ponte. Twist usa velocidade linear/angular; Ackermann
usa velocidade e ângulo geométrico, com modelo e entre-eixos declarados.

Configurações AMCL e mapas pertencem a ebva_localization. A conversão autônoma
física permanece condicionada à calibração do adaptador. Confira
[compatibilidade Humble](docs/humble-compatibility.md) e
[operação](../../docs/operation.md).

~~~bash
colcon build --packages-up-to ebva_navigation --symlink-install
colcon test --packages-select ebva_navigation
colcon test-result --verbose
~~~

Inspeção estrutural e build não demonstram localização ou navegação no veículo.
Footprint, frames, direção, limites e conversão devem corresponder à composição.

A solicitação única de salvamento ao slam_toolbox pertence a
ebva_localization/save_slam_map_once, com prefixo de saída explícito.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_navigation
source install/local_setup.bash
colcon test --packages-select ebva_navigation --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
