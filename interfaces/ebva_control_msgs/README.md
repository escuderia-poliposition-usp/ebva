# ebva_control_msgs — mensagens de controle e autoridade

## Objetivo

Definir tipos ROS compartilhados de observação, controle e atuação, mantendo significado das grandezas e procedência temporal. O pacote gera interfaces Python/C++; não executa controladores.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [msg/](msg/) | Definições de mensagens ROS. |
| [package.xml](package.xml), [CMakeLists.txt](CMakeLists.txt) | Dependências e instalação de recursos/entry points. |

## Interfaces

As definições completas estão em [msg/](msg/). `RcInput`, `SelectedControl` e `ActuatorCommand` ligam a autoridade; `DriveEffort`, `FtgPdCommand`, `LongitudinalEffortSteering` e famílias CBF mantêm contratos distintos. O catálogo abaixo descreve as famílias.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `std_msgs`, `rosidl_default_runtime`.
- **Build:** `ament_cmake`, `rosidl_default_generators`.

## Uso e configuração

| Mensagem | Grandeza e finalidade |
|---|---|
| RcInput | Par PWM atômico, stamp de recepção, validade do protocolo e estado RF |
| ActuatorCommand | eRPM e posição normalizada de servo [0,1], com tempo original |
| SelectedControl | Seleção STOP/MANUAL/AUTONOMY contendo os payloads das fontes |
| TrackClassificationStamped | Classificação estampada usada pelo supervisor |
| DriveEffort | Par de esforços adimensionais identificado por contract |
| FtgPdCommand | Saída FTG/PD e os dois headers consumidos |
| LongitudinalEffortSteering | Esforço longitudinal adimensional e ângulo geométrico em rad |
| FrenetLateralErrorStamped | Erro de folga lateral identificado por referência, corpo e ponto |
| AccelerationYawRateStamped | Aceleração m/s² e yaw rate rad/s do nominal autônomo |
| AccelerationSteeringAngleStamped | Aceleração m/s² e ângulo geométrico rad do nominal |
| CbfYawRateCommand | Saída do modelo CBF yaw_rate com procedência de três entradas |
| CbfYawRateMarginCommand | Saída do modelo CBF yaw_rate_margin |
| CbfSteeringTangentCommand | Aceleração e tangente de direção do modelo steering_tangent |

### Autoridade e atuação

RcInput preserva o tempo de recepção dos dois canais; o protocolo serial atual
só pode informar RF_UNKNOWN. ActuatorCommand não representa velocidade física.
SelectedControl mantém os stamps originais e a seleção do par inteiro.
Aquisição, autoridade e backend pertencem a ebva_vehicle_control.

### Classificação e esforço do supervisor

TrackClassificationStamped contém header, valid, track_type, position,
track_probability, track_entropy, position_probability, position_entropy e
curvature_feature. header é o tempo original da observação no frame do modelo.
O receptor verifica validade, finitude, domínios e expiração; não há parser
textual provisório ou produtor de classificação implícito.

Classes: straight, left_curve, right_curve, intersection, walls.
Posições: left, middle, right. Probabilidades são [0,1]; entropias são
não negativas. curvature_feature conserva a escala do método, sem conversão
implícita para unidade SI. O README de ebva_supervisor descreve a política
de idade, estados e seleção.

DriveEffort mantém um par atômico e o header da medida. Os contratos do
supervisor são supervisor_pi_straight_v1, supervisor_p_curve_v1,
supervisor_p_intersection_v1, supervisor_wall_recovery_v1 e
supervisor_straight_rule_v1. Longitudinal e direção são esforços normalizados,
com direção positiva para a direita nessas leis.

O contrato frm_point_sample_v1 também usa DriveEffort, com semântica própria.
Os adaptadores verificam o identificador do contrato, frame/idade e seu mapa
software_fixture específico. Um mapa de fixture não é calibração física.

### FTG/PD

FtgPdCommand identifica o contrato ftg_pd_sample_v1 e preserva o header do scan
e feedback_header da direção observada. Throttle não é velocidade comprovada;
o esforço PD não é um ângulo final ou um domínio normalizado presumido.
O adaptador preserva a validade da entrada mais antiga.

### Modelos CBF

As mensagens de nominal identificam unidades e modelo. As saídas CBF incluem
os headers nominal, scan e velocidade originais, o child frame da velocidade
e a componente longitudinal consumida. O tipo identifica o modelo; não há
troca silenciosa por um seletor genérico. Consumidores conferem as três idades
e o corpo/ponto declarado. O header de Odometry identifica o pai da pose.
Nenhum campo demonstra por si só a calibração eRPM/servo ou segurança física.

### Trajetória

LongitudinalEffortSteering distingue esforço de velocidade ou aceleração e
direção geométrica de yaw rate. Contratos:
waypoint_control_carrot_v1, waypoint_control_stanley_v1,
waypoint_control_pure_pursuit_v1 e lidar_frenet_pp_v1.

FrenetLateralErrorStamped preserva reference_id, body_frame_id, model_point e
contract_id; folga lateral não é automaticamente um erro cartesiano.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_control_msgs
source install/local_setup.bash
ros2 interface show ebva_control_msgs/msg/ActuatorCommand
~~~

Confira geração de interfaces e reconstrua consumidores ao alterar campos/tipos. Não há suíte própria; a semântica é exercitada nos produtores/consumidores.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
