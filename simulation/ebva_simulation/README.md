# ebva_simulation — fontes sintéticas e modelo cinemático

## Objetivo

Fornecer imagens sintéticas e modelo cinemático para exercícios/testes sem dispositivos. Permite escolher a fonte artificial independentemente do detector/controlador.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_simulation/](ebva_simulation/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [examples/](examples/) | Exemplos de composição/visualização. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [sim_kinematic.py](ebva_simulation/sim_kinematic.py): modelo bicicleta e TF.
- [hough_camera_fixture.py](ebva_simulation/hough_camera_fixture.py): fonte de imagem artificial.
- [kinematic.yaml](config/kinematic.yaml): hipóteses/ganhos do modelo.

## Interfaces

`synthetic_camera_publisher`, `hough_synthetic_camera`, `hough_camera_fixture`: imagens artificiais (`Image`). `sim_kinematic` recebe `Float64` nos tópicos absolutos `/commands/motor/speed` e `/commands/servo/position`, e publica TF. `kinematic.launch.py` seleciona `config/kinematic.yaml`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_lane_detection](../../perception/ebva_lane_detection/README.md).
- **ROS e bibliotecas externas:** `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `cv_bridge`, `python3-numpy`, `python3-opencv`, `std_msgs`, `ament_index_python`, `nav2_map_server`, `lifecycle_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

Fontes de câmera reutilizam geometria de ebva_lane_detection. AutoDRIVE tem outro pacote; Gym permanece futuro.

## Uso e configuração

| Interface | Função |
| --- | --- |
| synthetic_camera_publisher | Publica cena sintética para o pipeline de faixas |
| hough_synthetic_camera | Entrada de geração sintética para comparações Hough |
| hough_camera_fixture | Publica uma imagem artificial de faixas em /camera/image_raw |
| sim_kinematic | Modelo cinemático com parâmetros explícitos |

kinematic.launch.py fornece o modelo isolado, com params_file/default em
config/kinematic.yaml. Ele não calibra o carro.

~~~bash
ros2 launch ebva_simulation kinematic.launch.py
~~~

A seleção de uma fonte para testes é explícita. Imagens/modelos sintéticos
permitem verificar contratos de software, sem demonstrar realismo de câmera,
dinâmica ou precisão física.

A integração AutoDRIVE pertence a ebva_autodrive. A integração F1TENTH Gym é
uma capacidade futura descrita no [roadmap](../../docs/roadmap.md).
O recurso sim_navigate.rviz pode ser usado para visualização de uma composição
preparada, sem iniciar componentes por si.

~~~bash
colcon build --packages-up-to ebva_simulation --symlink-install
colcon test --packages-select ebva_simulation
colcon test-result --verbose
~~~

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_simulation
source install/local_setup.bash
colcon test --packages-select ebva_simulation --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Não há suíte test/ própria nesta revisão. As fontes são exercitadas por ebva_system_tests; retorno sem testes não demonstra validação do modelo.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
