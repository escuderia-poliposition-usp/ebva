# ebva_autodrive — ponte ros do AutoDRIVE

## Objetivo

Conectar ROS 2 ao simulador externo AutoDRIVE por Socket.IO e oferecer teleop. Preserva protocolo e avisos BSD dos Tinker Twins; simulador gráfico e controladores são componentes separados.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_autodrive/](ebva_autodrive/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [autodrive_bridge.py](ebva_autodrive/autodrive_bridge.py): servidor e callbacks.
- [config.py](ebva_autodrive/config.py): tópicos/tipos do protocolo.
- [teleop_keyboard.py](ebva_autodrive/teleop_keyboard.py): comandos interativos do simulador.

## Interfaces

`autodrive_bridge` abre servidor (`bind_address`, `port`, default 4567). `/autodrive/f1tenth_1/*` transporta sensores/estado e comandos `Float32` adimensionais. `teleop_keyboard` comanda o simulador; o reset `Bool` está em `/autodrive/reset_command`, fora do prefixo do veículo.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `std_msgs`, `nav_msgs`, `geometry_msgs`, `python3-numpy`, `python3-yaml`, `sensor_msgs`, `cv_bridge`, `tf2_ros`, `python3-pil`, `python3-gevent`, `python3-socketio`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

O backend adicional está em [requirements-autodrive.txt](requirements-autodrive.txt): Socket.IO/gevent/gevent-websocket. Simulador e protocolo compatíveis são preparados à parte.

## Uso e configuração

Os fontes da ponte/configuração/teclado conservam os avisos BSD dos Tinker Twins.

### Dependências e execução

O ambiente requer ROS Humble/Python 3.10, cv_bridge, tf2_ros,
NumPy e Pillow. requirements-autodrive.txt declara o backend Python Socket.IO,
gevent e gevent-websocket. Instale essas dependências no ambiente Python escolhido
para o simulador; elas não são necessárias para importar a configuração de tópicos.

~~~bash
ros2 run ebva_autodrive autodrive_bridge
ros2 run ebva_autodrive teleop_keyboard
~~~

A ponte abre o servidor somente ao executar main. bind_address (vazio por padrão,
mantendo a escuta original em todas as interfaces) e port (4567) são parâmetros ROS.
Importar o módulo não abre porta nem cria um nó. O teclado exige terminal
interativo; encerramento publica zero nos comandos do simulador.

Dados simulados são publicados sob /autodrive/f1tenth_1: câmera, LiDAR, IPS,
IMU, velocidade, encoders e métricas de volta. As entradas throttle_command e
steering_command são Float32 adimensionais em [-1,1]; reset_command é Bool.
Essas grandezas não são velocidade em m/s ou ângulo físico do carro.
A configuração conserva os tópicos e frames do protocolo original.

Um perfil escolhe ponte, percepção/estado, controlador e adaptação de comandos.
O controlador deve usar sua própria interface; o adaptador do simulador precisa
traduzir o contrato para o par normalizado do AutoDRIVE. Este pacote não atribui
calibração física ao veículo real.

O método de trajetória por curvatura pertence a ebva_path_control. Integração
funcional com F1TENTH Gym é trabalho futuro. Smoke tests offline de importação e
callbacks não qualificam uma sessão completa no simulador.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_autodrive
source install/local_setup.bash
colcon test --packages-select ebva_autodrive --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
