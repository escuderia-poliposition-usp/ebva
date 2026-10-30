# ebva_autodrive

Integração ROS com o simulador externo AutoDRIVE. Mantém a ponte Socket.IO,
configuração dos tópicos F1TENTH e teleop do simulador. Não inicia o simulador
gráfico, não fornece um controlador de trajetória e não publica comandos VESC.

Os fontes da ponte/configuração/teclado conservam os avisos BSD dos Tinker Twins.

## Dependências e execução

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
