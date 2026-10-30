# Waypoint Pure Pursuit: alvo cinemático cartesiano

Seguimento de referência XY ordenada por projeção e interseção de lookahead.
É distinto do CSV/Frenet por avanço escalar, do centralizador LiDAR, dos modos
waypoint_control e do diagnóstico preview.

## Cálculo e interface

O planner combina as operações:
projeção no segmento, interseção com círculo/fechamento, curvatura média,
velocidade por curvatura, atuação, quaternion→yaw, seleção de alvo e plan.
O construtor recebe a referência em memória; command concentra o cálculo. Não há arquivo, YAML, ament, ROS ou display
no núcleo. O recurso é carregado/validado uma vez pelo nó, antes dos publishers.

Parâmetros do modelo original, sem nova medição do veículo:

- Entre-eixos0,15597534362552312+0,15875=0,3147253436255231m.
- Lookahead0,5m, velocity_gain0,5, janela de curvatura5 de cada lado.
- Curvatura média usa magnitude unsigned da mudança angular entre segmentos.
- speed=min(12,sqrt((0,8*9,81)/(curvature+0,00001)))*0,5, até6m/s.
- y_local=[-sin(yaw),cos(yaw)]·(target-position),
  delta=atan(wheelbase*2*y_local/lookahead²).

A velocidade é **alvo do modelo**. O legado publicava speed/20 em throttle e
-delta em steering, separados e sem header. A nova interface expõe speed[m/s]
e delta[rad] em AckermannDriveStamped, por decisão explícita de contrato SI.
Retirar20 e a inversão da borda não demonstra equivalência física a throttle,
eRPM, servo ou ângulo medido. O pacote não fornece mapa de atuadores.

O sinal FLU é demonstrado por geometria: com yaw=0, alvo à esquerda tem
 y_local>0 e delta>0; com yaw=pi/2 o vetor lateral aponta para -x e um alvo
à esquerda novamente dá delta>0. Casos espelhados dão delta<0. O teste compara
esses cenários com as saídas originais, e não apenas o nome da variável.

O [fixture de regressão](../test/data/waypoint-pp-regression.json) exercita
esquerda/direita, orientação girada, alinhamento, vértice, fechamento, heading
inverso, fallback distante, limite de reacquisição e poses da referência
distribuída. Os resultados numéricos são verificações do método, não calibração
ou dados de condução do veículo.

## Referência, origem, fechamento e limitações do locus

CSV sem cabeçalho, exatamente duas colunas x,y em metros; pelo menos três
vértices distintos e o primeiro ponto repetido **exatamente** no fim. Todos os
valores e comprimentos dos segmentos devem ser finitos; segmentos adjacentes
nulos são rejeitados. Repetir primeiro/final para fechar é permitido; repetir
um ponto adjacente é outro caso. Na fonte original esse segundo caso produziu
projeção NaN por divisão por comprimento zero, reproduzida antes da guarda.
Não há ordenação, deduplicação, fechamento ou transformação silenciosa.

O CSV distribuído tem3000 pontos e fechamento explícito.
Os recursos incluem a configuração do método. sx/sy/stheta do YAML não eram
aplicados no cálculo ROS; não alinham automaticamente a pose ao caminho.
O novo contrato declara path_origin=shared_reference_frame: posição e referência
já usam o mesmo frame/origem SI antes de chegar ao controlador. path_direction
é csv_order; mudar yaw não reordena o recurso nem transforma o algoritmo em ré.

A projeção escolhe o segmento globalmente mais próximo, sem progresso temporal.
Autointerseções, segmentos próximos e trajetórias sobrepostas podem trocar o
ramo escolhido. O contrato não prova que um caminho é simples, livre de obstáculos
ou adequado ao veículo. O suporte ao wrap e o ajuste geométrico histórico de
1e-6 nos extremos das interseções foram preservados por equivalência.

Sem interseção de lookahead, o fallback mira o ponto projetado, com limiar de
reacquisição herdado1e9. Isso pode produzir alvo positivo muito longe do caminho;
não foi inventado um limite físico de afastamento nesta rodada. No limite de
reacquisição a saída pode ser zero. Não existe saturação final nova da direção;
perfis devem declarar limites e rejeitar valores fora da composição qualificada.
O locus de uso do veículo precisará de cenários/limites próprios antes de promoção.

## Corpo, pose, tempo e saída

Entrada PoseWithCovarianceStamped no tópico relativo pose. Consome XY e yaw;
a covariância não governa a lei e não foi convertida em garantia de qualidade.
O frame da pose precisa ser path_frame_id. **A mensagem não contém child_frame_id**:
corpo e ponto são afirmações da composição pose_contract, body_frame_id e
model_point. O nó não deduz corpo a partir do frame pai da pose.

Um perfil pode usar odometry_pose quando a odometria se refere ao mesmo corpo,
ponto, referência e origem; esse adaptador copia os dados e não faz TF. Relações
com base_link/câmera/eixo traseiro precisam ser explícitas; nenhum frame é
renomeado para substituir uma transformação física.

Ackermann recebe stamp original exato da pose e frame_id do corpo/ponto do
comando, não o pai da pose. Esse header expressa um comando corporal e não uma
pose mundial renomeada. Uma medida válida gera um par atômico; não há heartbeat.

Time exige sec>=0 e0<=nanosec<1e9 antes da soma. Pose/quaternion precisam ser
finitos e quaternion unitário. MeasurementLease compartilhado verifica frame,
idade original<200ms, tolerância futura20ms, duplicatas e reordenação. O prazo
monotônico impede prolongar a mesma medida com clock pausado. Prazo candidato
é confirmado somente depois do cálculo finito; falhas numéricas não renovam lease.

Sem entradas o nó deixa de publicar. Parada/expiração efetivas pertencem ao
adaptador, autoridade RC e backend. Após lacuna, uma pose nova pode retomar o
cálculo: o núcleo não possui progresso acumulado, PID ou memória longitudinal.
O encerramento destrói o nó e usa try_shutdown idempotente; erros arbitrários
continuam propagados após limpeza.

## Parâmetros e composição

Obrigatórios: csv_path, path_frame_id, body_frame_id, model_point=rear_axle ou
software_fixture_rear_axle, path_contract=ordered_closed_xy_si_v1,
path_origin=shared_reference_frame, path_direction=csv_order e
pose_contract=cartesian_body_pose_si_v1. Não há ponto/frame físico presumido.

Opcionais: observation_timeout=0,2, future_slack=0,02, pose_topic=pose e
command_topic=ackermann_command. São lidos ao iniciar. QoS reliable/volatile,
keep_last1. DiagnosticArray relativo control/path_status é publicado por entrada,
com header original, status.name=waypoint_pp, accepted ou motivo de rejeição,
body_frame e campos Time brutos. Diagnóstico não renova comando.

```bash
ros2 launch ebva_path_control select.launch.py method:=waypoint_pp \
  csv_path:=/caminho/reference.csv path_frame_id:=fixture_odom \
  body_frame_id:=fixture_rear_axle_flu model_point:=software_fixture_rear_axle \
  path_contract:=ordered_closed_xy_si_v1 path_origin:=shared_reference_frame \
  path_direction:=csv_order pose_contract:=cartesian_body_pose_si_v1
```

Esse launch inicia somente o controlador. O recurso de fixture instalado fica
em data/waypoint_pp_fixture/reference.csv; config/waypoint_pp_software.yaml contém
as afirmações da fixture, sem csv_path implícito. Perfis com autoridade pertencem ao bringup; o adaptador consumidor deve declarar
uma faixa compatível com o alvo. Limites sintéticos não são calibração física.

## Testes e limites

A suíte verifica regressão numérica, geometrias FLU, domínio, referências,
segmentos, Time, frame, quaternion, finitude, lease, relógio pausado, recuperação,
headers, silêncio e limpeza. O pacote utiliza dependências ROS/Python declaradas,
sem importar ferramentas de outro checkout.

[Contratos dos métodos](path-method-contracts.md) descrevem alternativas distintas.
Testes sintéticos não qualificam seguimento, velocidade ou calibração do veículo.
