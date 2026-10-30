# ebva_lane_keeping — controle por faixas

O executável lane_keeping recebe LaneBoundaries e calcula comando Ackermann.
O launch select.launch.py seleciona o núcleo v2; v1_compat true seleciona os
parâmetros da variante anterior. A instalação não inicia detector ou atuação.

~~~bash
ros2 launch ebva_lane_keeping select.launch.py
~~~

## Entrada, geometria e saída

LaneBoundaries usa x para frente e y para a direita no modelo de câmera.
A composição deve declarar expected_lane_frame e a correspondência geométrica;
não presuma FLU por causa do nome de frame.

A saída é Ackermann em m/s e rad. command_frame e command_topic declaram sua
ligação ao consumidor; o default de tópico é /ackermann_cmd. O controlador
não inicia mux ou VESC. Os perfis escolhem detector, método, adaptador e
autoridade conforme seus contratos.

## Validade e ciclo de vida

O wrapper exige stamp original fresco/crescente antes de chamar o núcleo.
Watchdog combina prazo monotônico e idade original, sem renovação por
duplicatas ou relógio de replay parado. Reset limpa o lease; reinicie o
perfil para iniciar outra época de replay.

FSM, perdas de linha, latch, reset, watchdog e fallback de velocidade têm
testes de software. As configurações de fixture não medem câmera, montagem,
escala, centro ou resposta física do veículo.

## Build e testes

~~~bash
colcon build --packages-up-to ebva_lane_keeping --symlink-install
colcon test --packages-select ebva_lane_keeping
colcon test-result --verbose
~~~

A campanha lane_control_replay pertence a ebva_system_tests e utiliza entradas
sintéticas com adaptador/autoridade/backend. Veja [testes](../../docs/testing.md),
[interfaces](../../docs/interfaces.md) e
[atribuições](../../THIRD_PARTY_NOTICES.md).
