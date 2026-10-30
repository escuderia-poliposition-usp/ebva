# ebva_navigation — navegação e ponte cinemática

Interfaces ROS Humble:

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
