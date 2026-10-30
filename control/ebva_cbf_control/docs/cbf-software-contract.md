# Modelos CBF e contrato da composição de software

O pacote mantém três modelos de filtro CBF: yaw_rate, yaw_rate_margin e
steering_tangent. Cada um conserva suas equações e seu domínio. A composição
CbfSoftwarePipeline recebe observações independentes e aplica um gate de
admissibilidade antes de disponibilizar um comando.

A entrada nominal é autônoma. Aquisição RC, autoridade e conversão para
atuadores pertencem a [ebva_vehicle_control](../../ebva_vehicle_control/README.md).
O CBF não substitui a seleção manual prioritária.

## Modelos e unidades

| Propriedade | yaw_rate | yaw_rate_margin | steering_tangent |
|---|---|---|---|
| Finalidade | Headway com yaw rate | Headway com margem espacial | Bicicleta com tangente de direção |
| Nominal longitudinal | Aceleração m/s² | Aceleração m/s² | Aceleração m/s² |
| Nominal lateral | Yaw rate rad/s | Yaw rate rad/s | Ângulo geométrico rad |
| Segunda variável do QP | Yaw rate rad/s | Yaw rate rad/s | tan(delta), adimensional |
| Limite lateral interno | ±30/180 = ±1/6 | ±1/6 | ±0,4 |
| Margem Dmin | 0 | 0,5 m | 0,5 m |
| Segundo coeficiente Lgh | -Td*v*sin(psi) | -Td*v*sin(psi) | +Td*v²*sin(psi)/L |
| Termo adicional Lfh | Nenhum | Nenhum | -Td*v²*sin²(psi)/R |

Os três QPs usam caixa longitudinal de ±3 m/s². O limite lateral ±1/6 é literal;
não deve ser trocado por ±pi/6 com base em comentários antigos sobre graus.
O modelo tangent recebe nominal até ±pi/9, convertido explicitamente para
tan(delta), e o QP usa a caixa ±0,4. Esses dois limites não são equivalentes.

O filtro interpola 20 raios na região frontal ±pi/2:

~~~text
Td = 0,5*cos(psi) + 0,1
gamma = 0,7 + 0,3*sin(psi)
alpha(h) = gamma*h³
L = 0,33 m
~~~

Obstáculos são estáticos no modelo. L e os demais números são parâmetros do
modelo implementado; sua presença no código não constitui medição do veículo.

## Solver e gate de admissibilidade

O solver Hildreth executa 38 iterações e não devolve um certificado de
convergência ou inviabilidade. Um resultado finito pode violar restrições.
Os casos de regressão numérica incluem:

- yaw_rate com obstáculo lateral: max(Au-b) de aproximadamente 0,005492 à
  esquerda e 0,020439 à direita. A testemunha [-3,w_retornado] tem resíduo 0;
  nesses casos o QP é viável, mas a resposta retornada é inadmissível.
- v=4 m/s e todos os raios em 0,1 m: existe restrição individual incompatível
  com qualquer comando da caixa. Os resíduos retornados são aproximadamente
  10,845, 17,822 e 49,773, respectivamente, nos três modelos.

CbfKernel mantém raw_control para diagnóstico e fornece command somente se
todos os valores forem finitos e max(Au-b)<=1e-6. O estado
infeasible_box_bound exige uma testemunha algébrica: para pelo menos uma linha,
min(A_i*u) na caixa supera b_i+tolerância. É um teste suficiente nesses casos,
não um decisor completo de inviabilidade. constraint_violation descreve as
demais respostas que não satisfazem as restrições.

Não há fallback que transforme uma rejeição em comando aceito. O gate verifica
as desigualdades implementadas; não demonstra otimalidade, estabilidade,
correção física do modelo ou segurança após conversões posteriores.

## Entradas da composição pura

CbfSoftwarePipeline exige o modelo e três observações:

- ScanSample: distâncias em m, ângulos em rad, aquisição e frame originais,
  geometria coerente e raios finitos dentro do intervalo declarado.
- SpeedSample: velocidade longitudinal em m/s, com o frame da base do twist
  identificado por child_frame_id; o pai da pose é uma informação diferente.
- YawRateNominal para os modelos yaw: aceleração m/s² e yaw rate rad/s.
  SteeringAngleNominal para tangent: aceleração m/s² e ângulo até ±20°.

A preparação de entrada rejeita NaN/inf em vez de lhes atribuir range_max.
Dados reais precisam estar no corpo FLU e ponto de referência declarados.
O pacote não infere extrínsecos, não transforma LiDAR por renomear frames e
não qualifica outros campos de uma mensagem Odometry.

## Tempo e atomicidade

MeasurementLease, em ebva_control_contracts, mantém tempo ROS original e prazo
monotônico independente para cada fonte. Os defaults são idade <200 ms,
tolerância futura de 20 ms e diferença máxima entre aquisições de 80 ms.

Duplicatas, reordenação e entradas rejeitadas não renovam o lease. Um scan
inválido não substitui o último válido; este só pode ser reutilizado enquanto
seu próprio prazo não vencer. Relógio ROS pausado não prolonga a observação.
As portas ROS também detectam regressão observada do relógio e exigem recriação.

Somente nominal novo válido dispara avaliação; não há heartbeat de comando.
SoftwareCommand conserva o modelo, o par indivisível e os três headers.
validity_header é o header da aquisição mais antiga, limite conservador de
validade, e não um identificador único de ciclo. Rejeições produzem status
observável e command=None. O núcleo não integra aceleração.

## Interfaces ROS e adaptação

As portas estampadas são descritas nos contratos de
[yaw_rate](yaw-rate-ros-contract.md) e
[margin/tangent](alternate-model-ros-contracts.md). Elas usam tipos distintos
para evitar equivalência acidental entre omega, delta e tan(delta).

Uma conversão aceita pelo QP ainda pode deixar de satisfazer A*u<=b depois de
um corte ou projeção. O adaptador tangent rejeita pares não representáveis em
seu limite angular; não corta silenciosamente a solução. Mapas de esforço,
velocidade, eRPM e servo são responsabilidades separadas.

Os leitores RC e pontes VESC antigos não fazem parte do pacote publicável.
Os executáveis *_filter e *_stamped mantêm suas interfaces específicas; os
contratos estampados são a fronteira com validade e procedência explícitas.

## Uso e testes

Em ROS Humble/Python 3.10, com as dependências declaradas:

~~~bash
colcon build --packages-up-to ebva_cbf_control --symlink-install
source install/local_setup.bash
colcon test --packages-select ebva_cbf_control
colcon test-result --verbose
~~~

Com ebva_system_tests instalado, o replay em memória pode ser executado sem
criar nós, abrir dispositivos ou publicar comandos:

~~~bash
ros2 run ebva_system_tests cbf_pipeline_replay --output /tmp/cbf-pipeline.json
~~~

Ele verifica aprovação e procedência, expiração independente com clock pausado,
duplicatas, recuperação, frames, skew, rejeição de nominal antigo e QP
inadmissível. Os testes numéricos comparam resultados e barreiras com tolerâncias
próprias; pequenas diferenças entre processos BLAS não alteram o gate de 1e-6.

Consulte [testes](../../../docs/testing.md),
[interfaces](../../../docs/interfaces.md) e
[autoridade](../../../docs/command-authority.md). Software aprovado não substitui
medição de velocidade, extrínsecos, resposta longitudinal, limites e calibração
necessários à composição física.
