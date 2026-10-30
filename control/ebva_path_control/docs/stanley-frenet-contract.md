# Stanley/Frenet de folgas, referência angular fixa

Controlador de folgas laterais com referência angular fixa. É distinto do modo
Stanley interno de waypoint_control.

## Modelo

O núcleo usa a lei abaixo. O [fixture de regressão](../test/data/stanley-frenet-regression.json)
acompanha seus testes numéricos.

```text
v = hypot(vx,vy)
theta_e = wrap(yaw - 0)
e = -lateral_error_m
delta = clip(theta_e + atan(1.2*e/(v+.1)) - .8*e, -.4, .4)
speed = -.6
```

O alvo novo declara speed em m/s assinado e delta em rad positivo à esquerda,
conforme Ackermann. A numeração e os sinais originais são preservados. Isso é
uma decisão explícita de interface cinemática; não prova tracking físico, nem
transforma -.6 em demonstração de defeito ou em homologação de marcha ré.
O módulo do feedback hypot ignora sinal de vx; permanece uma particularidade do
modelo a avaliar em cenários físicos/simulados adequados, sem corrigi-la só por
ser diferente de outro Stanley. A referência angular é zero fixo no frame de
referência, sem tangent heading variável ou projeção sobre caminho geral.

A entrada lateral mede meia diferença de folgas esquerda-direita:
(d_left - d_right) / 2. Ela não representa automaticamente erro cartesiano de
localização. O produtor deve declarar geometria, referência, corpo e instante
da medida; remapear o nome de um tópico não fornece essas informações.

## Entrada e saída

| Porta relativa padrão | Tipo | Contrato |
| --- | --- | --- |
| odometry | nav_msgs/Odometry | Orientation no reference_frame_id; twist.linear.x/y no child_frame_id=body_frame_id. Corpo/ponto precisam corresponder ao modelo. |
| lateral_error | ebva_control_msgs/FrenetLateralErrorStamped | Header original da medida no frame de referência, reference_id/corpo/ponto/contract_id explícitos e lateral_error_m em m. |
| ackermann_command | ackermann_msgs/AckermannDriveStamped | Par speed/steering atômico, stamp exatamente da mais antiga das duas medidas, frame de comando igual ao corpo declarado. |
| control/path_status | diagnostic_msgs/DiagnosticArray | Motivo de aceite/rejeição e ambos headers aceitos originais: sec/nanosec/frame, referência, corpo e ponto. |

`reference_contract` e `contract_id` devem ser
`stanley_frenet_clearance_fixed_zero_v1`. O significado é meia diferença de
folgas esquerda-direita em metros, referência angular fixa zero e corpo/ponto
explicitamente afirmados pela composição. Não se pode renomear um header LiDAR
bruto para map: um produtor real precisará justificar montagem, alinhamento,
referência das folgas e tempo efetivo. A fixture usa somente medidas sintéticas
com essa relação declarada. O produtor externo deve satisfazer esse contrato.

Todos os parâmetros abaixo são obrigatórios: reference_frame_id, body_frame_id,
model_point, reference_id e reference_contract. Pai e corpo são distintos;
model_point aceita rear_axle ou software_fixture_rear_axle. O pacote não faz TF
ou transformação de covariâncias/ponto. Os tópicos podem ser definidos por
odometry_topic, lateral_error_topic e command_topic. Não há tópicos absolutos
/drive ou Autodrive na nova borda.

## Tempo, validação e recuperação

Cada entrada possui MeasurementLease próprio: idade original ROS menor que200ms,
futuro tolerado até20ms e validade local monotônica limitada pelo tempo restante.
maximum_skew padrão80ms; timeout/futuro/skew precisam ser finitos, com futuro e
skew não negativos e menores que o timeout. Stamps ROS canônicos, estritamente
crescentes por fonte, frames/identificadores e grandezas consumidas finitas são
obrigatórios. Quaternion deve ser unitário dentro de1e-3 em norma ao quadrado.
Apenas orientation e vx/vy são consumidos da odometria; não se fabrica pose,
velocidade angular, covariância ou progresso de caminho.

Ambos callbacks podem produzir saída quando ambas entradas são válidas. Nenhum
timer publica ou renova comando. O menor stamp precisa avançar em relação à
última saída: uma fonte nova não renova a outra. O stamp final é copiado por
inteiros sec/nanosec do header original mais antigo; o status guarda os dois.
Sem odometria recebida, o novo caminho não comanda com yaw/v default. Silêncio,
NaN, referência errada ou amostras antigas não renovam leases. O consumidor
expira a última saída; o nó não depende de publicar seu próprio STOP no silêncio.
Dados novos e coerentes permitem recuperação, respeitando cursores anteriores;
relógio regressivo não faz medida antiga passar por nova.

## Testes e limites

As verificações numéricas preservam a lei e os sinais. Os testes ROS cobrem
ambas as ordens temporais das entradas, perdas independentes, NaN,
referência/corpo/stamps inválidos, repetição, recuperação e encerramento.
Sem odometria coerente, a implementação não deve comandar usando yaw ou velocidade
default. Um timer não renova o comando.

Ackermann serializa float32: -0.6 é recebido como -0.6000000238418579.
A tolerância de serialização é separada da precisão dos testes do núcleo.

O perfil integrado de software liga esta fonte à adaptação artificial Ackermann,
autoridade RC integral e backend desarmável. Ganhos100eRPM/(m/s),-.2servo/rad e
centro.5 não são calibração. RC prevalece sobre o par completo e autonomia só
retoma ao neutro com entradas válidas. O pacote não importa bringup/ferramentas
ou backend, e pode compilar em workspace isolado com dependências declaradas.

Preview tem função diagnóstica própria e não substitui o contrato de seguimento.
[Contratos de todos os métodos](path-method-contracts.md).
