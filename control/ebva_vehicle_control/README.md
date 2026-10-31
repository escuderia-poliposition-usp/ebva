# ebva_vehicle_control — aquisição RC, autoridade e atuação

## Objetivo

Ligar RC e fontes externas ao veículo por etapas explícitas: aquisição, conversão, seleção de autoridade e backend VESC. Concentra prioridade RC, validade, habilitação e limites.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_vehicle_control/](ebva_vehicle_control/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [nodes.py](ebva_vehicle_control/nodes.py): aquisição, autoridade e backend ROS.
- [stamped_adapter.py](ebva_vehicle_control/stamped_adapter.py): base dos adaptadores estampados.
- [keyboard_adapter.py](ebva_vehicle_control/keyboard_adapter.py): teclado com validade e seleção.

## Interfaces

Aquisição `RcInput` → autoridade `SelectedControl` → backend eRPM/servo.
Adaptadores recebem os contratos dos controladores e entregam `ActuatorCommand`.
Os nomes abaixo são relativos: no perfil manual, recebem o namespace `/ebva`.

| Interface | Tipo e papel |
| --- | --- |
| `rc/input` | `ebva_control_msgs/RcInput`, aquisição para a autoridade |
| `control/autonomy/actuators` | `ActuatorCommand`, fonte externa para a autoridade |
| `control/selected` | `SelectedControl`, seleção e motivo para o backend |
| `commands/motor/speed` | `std_msgs/Float64`, eRPM |
| `commands/servo/position` | `std_msgs/Float64`, posição normalizada [0,1] |
| `control/set_autonomy_enabled` | `std_srvs/SetBool`, seleção do canal externo |
| `control/set_actuation_enabled` | `std_srvs/SetBool`, habilitação do backend |
| `control/status`, `control/backend_status` | `diagnostic_msgs/DiagnosticArray` |

Launches públicos: manual.launch.py, keyboard.launch.py, vesc_driver.launch.py,
wheel_odometry.launch.py e navigation_adapter.launch.py. params_file seleciona
a configuração do dono; teclado também aceita vehicle_params_file e key_params_file.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_contracts](../ebva_control_contracts/README.md), [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md).
- **ROS e bibliotecas externas:** `rclpy`, `geometry_msgs`, `key_teleop`, `ackermann_msgs`, `std_msgs`, `std_srvs`, `diagnostic_msgs`, `python3-serial`, `python3-yaml`, `launch`, `launch_ros`, `ament_index_python`, `vesc_driver`, `vesc_msgs`, `vesc_ackermann`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

VESC e key_teleop vêm de [dependencies.repos](../../dependencies.repos). Serial exige porta/permissões, teclado exige TTY e conversões físicas exigem calibração explícita.

## Uso e configuração

### Manual

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

### Prioridade e unidades

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

### Teclado com RC prioritário

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

keyboard.launch.py aceita params_file, vehicle_params_file e key_params_file.
Os três recursos pertencem ao pacote; a configuração EBVA não precisa ser
instalada dentro do fornecedor.

### Adaptadores autônomos

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

### Publicação direta de ensaio

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

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_vehicle_control
source install/local_setup.bash
colcon test --packages-select ebva_vehicle_control --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Testes/campanhas sintéticas não qualificam RF, watchdog, frenagem ou calibração.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
