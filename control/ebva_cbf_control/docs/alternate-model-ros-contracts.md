# Portas ROS dos modelos CBF margin e tangent

Cada produtor usa o kernel de seu modelo e um tipo de saída próprio.
As portas não escolhem um modelo por campo numérico, não misturam componentes
dos modelos e não publicam comandos finais de atuadores.

## Interfaces e unidades

| Executável / nó | Nominal | Saída |
|---|---|---|
| yaw_rate_margin_stamped / cbf_yaw_rate_margin | AccelerationYawRateStamped: a m/s² e omega rad/s | CbfYawRateMarginCommand: a m/s² e omega rad/s |
| steering_tangent_stamped / cbf_steering_tangent | AccelerationSteeringAngleStamped: a m/s² e delta rad | CbfSteeringTangentCommand: a m/s² e tan(delta) adimensional |

Ambos recebem perception/scan, LaserScan, e state/odometry, Odometry. Consomem
somente twist.twist.linear.x m/s no corpo/ponto declarado. O nominal default
é control/autonomy/nominal; os comandos são control/autonomy/cbf_yaw_rate_margin
e control/autonomy/cbf_steering_tangent.

scan_topic, velocity_topic, nominal_topic e command_topic configuram tópicos
relativos. QoS é keep-last 1/volatile; scan é best-effort e os demais reliable.

O nominal tangent recebe ângulo geométrico até ±pi/9 inclusive e o converte
explicitamente para tangente antes do QP. O nominal margin recebe a/omega
finitos; a projeção na caixa é responsabilidade do QP.

| Propriedade | yaw_rate_margin | steering_tangent |
|---|---|---|
| Variável lateral interna e de saída | omega rad/s | tan(delta), adimensional |
| Caixa QP | a ±3; omega ±1/6 | a ±3; tan(delta) ±0,4 |
| Dmin | 0,5 m | 0,5 m |
| Segundo Lgh | -Td*v*sin(psi) | +Td*v²*sin(psi)/L |
| Termo adicional Lfh | Nenhum | -Td*v²*sin²(psi)/R |

O produtor conserva o par QP aprovado pelo gate. Não aplica projeções/cortes
posteriores, integra aceleração ou converte para eRPM/servo. O teto positivo
0,24 de outras interfaces é uma projeção posterior, não um limite desse solver.

## Representabilidade e projeções posteriores

O nominal delta ±20° e a caixa tan(delta) ±0,4 são diferentes. Um resultado
filtrado pode exigir tangente além de tan(20°). Alterar uma solução aprovada
por um corte posterior não preserva automaticamente A*u<=b.

Um caso de regressão demonstra o efeito: v=0,2 m/s, raios 0,7 m para psi<-15°
e 8 m no restante, nominal [0,1;0], produzem QP [-1,045994497;-0,4] com resíduo
máximo 0. atan(-0,4)=-21,8014° cortado para -20° corresponde a tangente
-0,3639702343 e produz resíduo máximo 0,0007852640 no mesmo A/b, acima de 1e-6.
Com ângulos e raios serializados em float32, o resultado continua aceito com
tangente -0,4; coeficientes e aceleração refletem a geometria amostrada.

O adaptador tangent em ebva_vehicle_control rejeita o par que exige ângulo
além de ±20°, com unrepresentable_steering. Não corta silenciosamente o vetor
aceito. O produtor conserva o QP e sua procedência; a cadeia expira quando
não recebe um comando novo representável. Rejeitar por representabilidade
não altera o kernel nem demonstra segurança física.

## Procedência, corpo e validade

header é cópia exata do nominal; scan_header e velocity_header são cópias das
medidas usadas. O pai de velocidade não é relabelado para o corpo.
velocity_child_frame_id e longitudinal_velocity_mps preservam a base e o valor
consumidos. O consumidor deve limitar a validade pela aquisição mais antiga.

São obrigatórios control_frame, velocity_parent_frame,
model_reference_point=rear_axle_center e scan_geometry=coincident_aligned.
Scan e nominal já devem estar no corpo FLU indicado; velocidade deve ter
o pai indicado e child_frame_id igual ao corpo. L=0,33 m é o parâmetro do
modelo. Não há inferência de posição ou transformação do LiDAR real.

YawRateMarginPort e SteeringTangentPort reutilizam a observação e checagem
de scan/velocidade de YawRatePort. Cada classe fixa seu CbfSoftwarePipeline
e seus tipos ROS. Um scan rejeitado não substitui dados válidos nem renova
seu prazo.

Cada fonte possui stamp e lease monotônico independente. Defaults:
idade <200 ms, futuro <=20 ms e skew <=80 ms. Time deve ser canônico e
positivo; duplicatas/reordenação não renovam as entradas. A configuração
exige future_tolerance_s < timeout_s.

Relógio ROS pausado não prolonga dados; regressão observada trava a porta até
recriá-la. Somente nominal novo aceito dispara cálculo. Não há timer,
heartbeat de comando ou integração longitudinal em silêncio.

~/status publica JSON por entrada com model, source, accepted, reason,
command_count, clock_reset_required e resíduo quando disponível. Resultado
rejeitado não publica comando. O gate exige finitude e A*u<=b com tolerância
1e-6. Uma prova suficiente de inviabilidade de linha sobre a caixa é distinta
de constraint_violation; veja [o contrato dos modelos](cbf-software-contract.md).

## Uso e composição

Os produtores não abrem serial, publicam TF, recebem RcInput ou escrevem
motor/servo. O nominal é autônomo. A rota RC fica fora do filtro, selecionada
pela autoridade para o par completo, com retomada ao neutro válida.

Com ROS Humble/Python 3.10 e dependências declaradas:

~~~bash
ros2 run ebva_cbf_control yaw_rate_margin_stamped --ros-args \
  -r __ns:=/margin_fixture \
  -p control_frame:=fixture_rear_axle_flu -p velocity_parent_frame:=fixture_odom \
  -p model_reference_point:=rear_axle_center -p scan_geometry:=coincident_aligned
ros2 run ebva_cbf_control steering_tangent_stamped --ros-args \
  -r __ns:=/tangent_fixture \
  -p control_frame:=fixture_rear_axle_flu -p velocity_parent_frame:=fixture_odom \
  -p model_reference_point:=rear_axle_center -p scan_geometry:=coincident_aligned
~~~

Os nomes fixture representam observações sintéticas. Para uso real, o provedor
de estado e sensores deve cumprir o corpo/ponto e unidades contratados.

SIGINT/SIGTERM usam espera spin_once de até 50 ms. Nós são destruídos antes
de try_shutdown; sinais repetidos não interrompem a limpeza e erros reais
continuam propagados.

## Verificação e limites

A suíte verifica procedência exata, tipo por modelo, idade/frame/Time antes
do lease, prazos monotônicos independentes, clock regressivo, recuperação,
fontes inválidas e resultados QP inadmissíveis. Os testes DDS verificam
silêncio sem heartbeat, duplicatas e retomada. Os testes de ciclo de vida
cobrem os dois executáveis e propagação de erros após limpeza.

LaserScan.ranges é float32. Nos casos de regressão da borda ROS, a comparação
usa limite absoluto 2e-8 para representação de valores como 0,1 e 0,8 m.
Essa tolerância é separada do gate 1e-6 e da comparação numérica pura 1e-12.

Software aprovado não mede extrínsecos, velocidade, resposta longitudinal ou
calibração de atuadores. Referências:
[README do pacote](../README.md),
[contrato yaw_rate](yaw-rate-ros-contract.md),
[interfaces](../../../docs/interfaces.md),
[testes](../../../docs/testing.md) e
[autoridade](../../../docs/command-authority.md).
