# ebva_vehicle_control — aquisição, autoridade e atuação

O pacote separa aquisição RC, autoridade, adaptação de comandos e backend VESC.
Interfaces: manual.launch.py, keyboard.launch.py, vesc_driver.launch.py,
wheel_odometry.launch.py e navigation_adapter.launch.py. O argumento params_file
seleciona a configuração do dono.

## Manual

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

Manual real inclui RC, autoridade, backend e VESC. O default
config/vehicle/manual_core.yaml usa auto_enable_on_startup true: o backend
começa sem habilitação e aguarda grafo consistente e seleção MANUAL com RC
válido/recente e acelerador/direção no neutro. Habilita uma vez, com autonomia off.

Até habilitar, os diagnósticos incluem startup_waiting_for_graph e
startup_waiting_for_neutral_rc. Um pedido explícito a
control/set_actuation_enabled cancela a tentativa, inclusive data false.
Desarme posterior por serviço ou conflito não é revertido automaticamente.

Para exigir habilitação explícita, selecione false no YAML. O default genérico
do executável é false; a escolha true pertence à configuração manual.
Reinicie o launch para carregar alterações.

Sem check_only, este comando abre hardware e permite atuação:

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual
~~~

## Prioridade e unidades

RC válido assume o par motor/direção. Com autonomia off, neutro aplica zero e
centro configurado. Com autonomia on, voltar ao neutro retoma o comando externo
somente com entradas válidas. Perda de RC bloqueia autonomia. Reiniciar autoridade
não rearma o backend.

RcInput carrega canais em microssegundos e validade. ActuatorCommand é o par
eRPM/servo normalizado. SelectedControl informa modo, motivo e entradas.
Esses valores não são velocidade/ângulo físicos.

Portas, polaridades, centro e limites pertencem à configuração do veículo.
Não altere esses valores para tornar uma fixture aparentemente compatível.
Veja [autoridade](../../docs/command-authority.md).

## Teclado com RC prioritário

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=keyboard
~~~

O perfil usa key_teleop do fornecedor em terminal SSH interativo.
O adaptador conecta curses ao terminal controlador; use ssh -t quando
necessário. Setas comandam motor/direção; q encerra o perfil. Soltar teclas
retorna a zero após a janela do fornecedor.

RC deve estar válido/neutro na partida. Após backend habilitado e RC/teclado
neutros, o adaptador seleciona uma vez o canal externo por
control/set_autonomy_enabled. Não chama o serviço de atuação nem reativa esse
canal após desabilitação explícita. Voltar RC ao neutro só retoma teclado com
entradas válidas.

~~~text
key_vel (TwistStamped) → keyboard_adapter → control/autonomy/actuators
→ command_authority → control/selected → vesc_backend → commands/...
~~~

AUTONOMY nessa rota significa canal não-RC. A conversão usa eixos normalizados:
forward_rate 0.8 é 80% do erpm_limit, não 0.8 m/s. A direção interpola
centro/extremos respeitando limites. Stamp, frame, idade, eixos não suportados
e publicador concorrente são verificados.

Configurações deste pacote:

- config/vehicle/manual_core.yaml: portas, autoridade, partida e limites.
- config/keyboard_adapter.yaml: escalas, polaridades, lease e seleção inicial.
- config/key_teleop.yaml: taxas e saída estampada do fornecedor.

Keyboard.launch.py aceita params_file, vehicle_params_file e key_params_file.
Os três recursos pertencem ao pacote; a configuração EBVA não precisa ser
instalada dentro do fornecedor.

## Adaptadores autônomos

Métodos produzem contratos próprios: Ackermann, DriveEffort, FTG/PD,
esforço/ângulo ou modelos CBF. Adaptadores específicos ligam esses contratos
à autoridade; não existe uma conversão longitudinal universal.

O adaptador de navegação mantém qualified false e calibration_id vazio até
haver calibração definida. Adaptadores software_fixture aceitam somente o
propósito sintético declarado; seus ganhos não são calibração do carro.

Supervisor_effort_adapter recebe DriveEffort dos contratos `supervisor_<lei>_v1`,
valida frame/idade e utiliza mapeamento software_fixture. A campanha
supervisor_software o exercita com autoridade/backend sintéticos, sem driver.
Os perfis operacionais supervisor_* iniciam só o controlador e não habilitam
essa cadeia física implicitamente.

## Publicação direta de ensaio

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=direct_commands check_only:=true
~~~

Direct_commands inicia somente vesc_driver_node em /ebva. Não inclui RC,
autoridade, backend, teleop ou publicadores e não exige habilitação por serviço.
Carrega o bloco vesc_driver_node do mesmo YAML manual.

Durante sua execução, os tópicos finais aceitam std_msgs/Float64:
commands/motor/speed em eRPM e commands/servo/position normalizada.
O publicador de ensaio deve encerrar com longitudinal zero; Ctrl+C do
publicador não envia zero automaticamente.

Esse modo não oferece prioridade RC, lease ou desarme pelo backend.
Os limites internos do driver continuam aplicados. Não execute outra cadeia
de atuação ao lado. Abrir o driver pode reposicionar o servo.

## Build e testes

~~~bash
colcon build --packages-up-to ebva_vehicle_control --symlink-install
source install/local_setup.bash
colcon test --packages-select ebva_vehicle_control
colcon test-result --verbose
~~~

Testes/campanhas sintéticas não qualificam RF, watchdog, frenagem ou calibração.
