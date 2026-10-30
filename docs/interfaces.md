# Interfaces e significado dos dados

Cada ligação declara tipo, tópico, unidade/sinal, frame/ponto do modelo,
origem do timestamp, QoS e política de perda/expiração. O tipo ROS sozinho
não comprova intercâmbio entre implementações.

## Controle

| Tipo | Significado |
| --- | --- |
| AckermannDriveStamped | Velocidade m/s e ângulo geométrico de direção rad |
| Twist / TwistStamped | Velocidade linear m/s e angular rad/s no contrato cinemático |
| DriveEffort | Par adimensional atômico; contract identifica a semântica do produtor |
| LongitudinalEffortSteering | Esforço longitudinal adimensional e direção geométrica rad |
| FtgPdCommand | Saída numérica FTG/PD com duas medidas originais |
| CbfYawRateCommand / CbfYawRateMarginCommand | Aceleração e yaw rate, com proveniência das três entradas |
| CbfSteeringTangentCommand | Aceleração e tangente do ângulo no modelo específico |
| ActuatorCommand | Par atômico eRPM/servo normalizado |
| RcInput / SelectedControl | Aquisição RC e seleção de autoridade com validade/motivo |

Veja o [pacote de mensagens](../interfaces/ebva_control_msgs/README.md) e os
contratos dos produtores. O supervisor recebe classificação tipada/estampada
e produz DriveEffort. Sua característica de curvatura conserva a escala
declarada pelo método, sem atribuição implícita de unidade SI.

Os modelos CBF mantêm equações e estados próprios. A entrada nominal do modelo
tangente usa ângulo rad; a variável filtrada é `tan(delta)`. Preservar o
resultado do QP não demonstra segurança após conversões e limites de atuação.

## Coordenadas e referências

Grandezas físicas usam SI quando o contrato declarar medida física. O corpo
ROS segue x para frente, y para esquerda e z para cima:
[REP-103](https://www.ros.org/reps/rep-0103.html) e
[REP-105](https://www.ros.org/reps/rep-0105.html).

Alguns métodos de faixas têm x para frente e y para a direita, com origem
na câmera. Esse modelo precisa ser transformado para o corpo consumidor;
trocar `frame_id` sem transformar os pontos altera o significado.

Em Odometry, pose usa `header.frame_id` e velocidade usa `child_frame_id`.
Não crie velocidade ou covariância fictícia para preencher um tipo.
Path representa poses ordenadas; velocidade, curvatura e distância percorrida
exigem recursos/contratos próprios quando consumidas.

## Tempo e validade

Preserve timestamps de aquisição/geração. Adaptadores não renovam a idade só
por receber ou republicar uma entrada. Resultados com mais de uma medida
preservam e validam também suas entradas originais.

Verifique idade e silêncio local. Stamps ROS exigem política para /clock,
mensagens futuras, repetição e salto de relógio. Relógio monotônico permite
observar silêncio sem depender do relógio de replay.

QoS e filas compatíveis são necessários, mas não substituem expiração.
Dois Float32 sem sequência não comprovam que motor e direção pertençam
ao mesmo ciclo.

## Imagens, placas e profundidade

Detection2DArray oferece classe, score e caixa em pixels com header da imagem.
Uma detecção 2D não vira posição 3D por remapeamento.

RGB e profundidade exigem associação temporal, registro e calibração.
`32FC1` usa metros e `16UC1` milímetros no contrato de profundidade;
encodings ou dimensões incompatíveis são rejeitados. Profundidade Z da câmera
e distância euclidiana são grandezas diferentes.

Fixtures verificam unidades e transformações declaradas. Montagem, extrínsecos
e precisão do veículo exigem avaliação com dados reais.
