# Porta ROS do modelo CBF yaw_rate

O executável yaw_rate_stamped constrói CbfYawRateNode, com nome cbf_yaw_rate.
Ele recebe o nominal autônomo e as duas observações do modelo, publica somente
resultados aprovados pelo gate e mantém a procedência das três entradas.
Não abre serial, consulta TF, recebe RcInput ou publica motor/servo.

## Tópicos e unidades

| Tópico relativo | Tipo | Semântica |
|---|---|---|
| perception/scan | sensor_msgs/LaserScan | Distâncias m, ângulos rad, aquisição e frame originais |
| state/odometry | nav_msgs/Odometry | linear.x m/s no child_frame_id e ponto declarados |
| control/autonomy/nominal | ebva_control_msgs/AccelerationYawRateStamped | Par atômico de aceleração m/s² e yaw rate rad/s |
| control/autonomy/cbf_yaw_rate | ebva_control_msgs/CbfYawRateCommand | Par QP aceito, com procedências e velocidade usada |
| ~/status | std_msgs/String | JSON de aceitação/rejeição por entrada |

scan_topic, velocity_topic, nominal_topic e command_topic permitem selecionar
os tópicos de dados. QoS é keep-last 1/volatile; scan é best-effort e as demais
portas são reliable. Não há timer de republicação de comandos ou status.

CbfYawRateCommand.header é o header exato do nominal aceito. scan_header e
velocity_header preservam as aquisições efetivamente usadas. O header de
velocidade continua identificando o pai da pose; velocity_child_frame_id mantém
a base do twist, e longitudinal_velocity_mps mantém linear.x consumido.

A saída é [a,omega], com caixa [-3,3] m/s² e [-1/6,1/6] rad/s. Não há teto
positivo 0,24, integração longitudinal ou conversão implícita de aceleração em
velocidade/throttle. A adaptação posterior pertence a seu consumidor específico.

## Corpo, ponto e geometria

São obrigatórios quatro parâmetros:

| Parâmetro | Contrato |
|---|---|
| control_frame | Corpo FLU em que scan e nominal já estão expressos |
| velocity_parent_frame | Pai identificado por Odometry.header.frame_id |
| model_reference_point | rear_axle_center |
| scan_geometry | coincident_aligned |

Os defaults vazios impedem execução sem declaração de geometria. Scan e nominal
devem ter frame_id igual a control_frame. Odometry deve ter o pai declarado e
child_frame_id igual ao corpo. FLU significa x para frente, y para esquerda e
z para cima; o ponto do modelo é o centro do eixo traseiro.

Nos testes, o sensor é alinhado e coincidente com esse ponto por construção.
Isso não mede o LiDAR real ou autoriza substituir frame_id para simular TF.
Extrínsecos e transformação para o ponto do modelo são responsabilidades da
composição anterior a esta porta.

Somente twist.twist.linear.x é consumido. A aceitação não qualifica pose,
quaternion, covariâncias, outros componentes do twist, precisão ou deriva.
O provedor continua responsável por unidade, sentido e ponto da velocidade.

## Idade, leases e gate

Os headers são verificados em nanossegundos inteiros antes de atualizar leases:
stamp canônico positivo, frame esperado, idade menor que timeout_s e futuro até
future_tolerance_s inclusive. nanosec>=1e9 é rejeitado antes de somar o tempo.

| Parâmetro temporal | Default |
|---|---|
| timeout_s | 0,2 s |
| future_tolerance_s | 0,02 s |
| max_skew_s | 0,08 s |

Cada entrada tem MeasurementLease independente, com idade ROS original e prazo
monotônico. Duplicatas/reordenação não renovam dados. Relógio ROS pausado não
prolonga scan/velocidade, mesmo que cheguem nominais novos.

Regressão ROS observada entre callbacks trava a porta com clock_reset_required
até recriar o nó. Saltos sem callbacks não são observados; isso não substitui
um monitor global de relógio. A política exige future_tolerance_s < timeout_s.

Somente nominal novo válido dispara cálculo. Fontes rejeitadas não renovam
leases; a última fonte válida permanece utilizável até seu próprio prazo.
As três aquisições devem respeitar max_skew_s. Recuperação exige observações
novas das fontes vencidas; o nominal não renova scan nem velocidade.

YawRatePort adapta a representação ROS para CbfSoftwarePipeline. O gate mantém
accepted, constraint_violation e infeasible_box_bound conforme
[o contrato matemático](cbf-software-contract.md). Uma rejeição publica status
e nenhum comando; não troca solver ou cria um comando de fallback.

O JSON de status inclui source, accepted, reason, command_count,
clock_reset_required e resíduo numérico quando disponível.

## Uso independente

Construa ebva_control_msgs, ebva_control_contracts e ebva_cbf_control com suas
dependências ROS. Uma porta de teste, sem provedor de dispositivo, pode ser
iniciada assim:

~~~bash
ros2 run ebva_cbf_control yaw_rate_stamped --ros-args \
  -r __ns:=/cbf_software_fixture \
  -p control_frame:=fixture_rear_axle_flu \
  -p velocity_parent_frame:=fixture_odom \
  -p model_reference_point:=rear_axle_center \
  -p scan_geometry:=coincident_aligned
~~~

Sem as três entradas válidas não há saída. Os nomes fixture são declarações
de dados sintéticos, não defaults de montagem física. Para compor um perfil,
referencie a configuração do dono pelo
[orquestrador](../../../bringup/ebva_bringup/README.md).

SIGINT/SIGTERM permitem destruir os nós antes de encerrar ROS, com espera
spin_once máxima de 50 ms. Sinais repetidos não interrompem a limpeza;
exceções reais de callbacks continuam propagadas.

## Verificação e limites

A suíte cobre cópia exata dos três headers, par atômico, silêncio sem
republicação, duplicatas, expiração, retomada fresca, frames/child incorretos,
Time malformado, regressão de clock, skew e resultados QP inadmissíveis.

A aprovação do produtor não comprova a cadeia de conversão, autoridade e
atuação. Os modelos [margin e tangent](alternate-model-ros-contracts.md) têm
tipos e conversões próprios. Calibração longitudinal, extrínsecos, RF e
comportamento físico exigem verificação na composição escolhida.

Referências: [README do pacote](../README.md),
[interfaces gerais](../../../docs/interfaces.md),
[testes](../../../docs/testing.md) e
[autoridade de comandos](../../../docs/command-authority.md).
