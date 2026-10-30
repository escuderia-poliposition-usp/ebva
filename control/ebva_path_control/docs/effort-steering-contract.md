# Esforço longitudinal e esterçamento geométrico

Waypoint control e LiDAR/Frenet preservam modelos distintos. Conversões de esforço
para atuação exigem um adaptador com propósito e limites explícitos.

## Tipo e validade

LongitudinalEffortSteering contém header, contract, longitudinal_effort
adimensional e steering_angle_rad geométrico em rad. Ambos os valores são
float64 na interface. O header é cópia exata da medida usada:
Odometry no waypoint (frame pai) e LaserScan no LiDAR (frame do scan).
DriveEffort não foi reutilizado porque declara ambos os componentes adimensionais.

IDs exatos: waypoint_control_carrot_v1, waypoint_control_stanley_v1,
waypoint_control_pure_pursuit_v1 e lidar_frenet_pp_v1. A seleção de ID não muda
unidades: o conversor precisa conhecer o modelo, a finalidade e a calibração.

Cada medida aceita pode produzir um par atômico. Não existe timer que repita
comando nem troca do stamp pela hora de processamento. Header ROS canônico,
idade original (padrão200ms), tolerância futura20ms, ordem estrita e frame são
verificados; duplicata não renova lease mesmo com clock pausado. Diagnóstico é
publicado por medida, inclusive rejeitada. A autoridade/backend devem expirar
independentemente no silêncio; o controlador não promete frenagem por silêncio.

Tópicos relativos: effort_steering_command (saída), control/path_status
(diagnóstico). Um perfil liga a saída ao conversor escolhido, à autoridade RC e
ao backend exclusivos. O pacote não importa nem inicia esses componentes.

## Waypoint control

Entrada odometry (nav_msgs/Odometry). A referência CSV usa x_center,y_center
nomeados em metros; ordem das linhas e primeiro alvo posterior ao mais próximo
que alcance0,8m são preservados. Não fecha o laço por módulo, nem projeta em s.
Alvo atrás (x_v<=0) não gera comando. Corpo/ponto são afirmações explícitas da
composição; o controlador confere pai e child_frame_id, posição XY finita e
quaternion unitário. Covariância/twist não participam destas leis.

Parâmetros obrigatórios: reference_frame_id, body_frame_id distintos;
model_point=rear_axle ou software_fixture_rear_axle;
reference_contract=ordered_xy_si_v1;
reference_origin=shared_reference_frame;
reference_direction=csv_order;
pose_contract=cartesian_body_pose_si_v1.
control_mode seleciona carrot (padrão), stanley ou pure_pursuit.
observation_timeout e future_slack têm os padrões acima.

As três leis e parâmetros permanecem: lookahead0,8m, entre-eixos0,33m,
k_heading1,5, k_stanley2, throttle0,2 e clip de direção±0,41rad.
O Stanley usa erro angular do alvo e atan2(2*y_v,1), sem velocidade no
denominador. Não é substituído pelo Stanley/Frenet apenas pelo nome.

csv_path pode selecionar referência própria. Vazio resolve o recurso instalado
do **próprio** ebva_path_control em data/waypoint_debug/centerline.csv.
A consulta anterior a pp_controller_pkg estava incorreta e foi retirada.
Recursos existentes não tiveram bytes/permissões alterados.

Marcador opcional control/lookahead conserva target/frame e agora leva o
stamp da odometria e quaternion identidade válido; não é fonte de comando.
As únicas assinaturas públicas de leis antigas são referências ao núcleo puro.

## LiDAR Frenet

Entrada scan (sensor_msgs/LaserScan), QoS best_effort/depth1. Parâmetros
scan_frame_id e scan_contract=planar_body_flu_ranges_m_v1 são obrigatórios:
raios em metros, scan plano, eixos FLU paralelos ao corpo. Não há transformação
TF implícita nem calibração de extrínsecos criada.

Medianas dos setores abertos70–110° e -110–-70° são preservadas. Raios NaN/Inf
continuam ignorados; deve existir ao menos um retorno finito em cada lado.
Valores finitos consumidos fora de range_min/range_max são rejeitados.
Geometria exige incremento positivo, limites finitos/coerentes e range_min>=0.
Raios fora dos setores não alteram o resultado nem são reparados.

e_y=(d_left-d_right)/2; kappa=2*e_y/1,3²; delta=atan(0,26*kappa).
Throttle fixo0,4 permanece adimensional. Não existe CSV em execução.

Este método não consome odometria: medianas, e_y, delta e esforço longitudinal
dependem do scan. A cadência acompanha medidas novas, sem renovar o comando
por timer durante silêncio.

## Testes e limites

Regressões numéricas incluem saturação, curvas e alvos atrás. Testes verificam
configuração, recursos, valores, frames/tempo, duplicatas, silêncio, recuperação,
QoS, headers e encerramento. Os métodos waypoint_pp, CSV/Frenet e Stanley/Frenet
têm contratos independentes. Preview é diagnóstico.

Ganhos esforço→eRPM/ângulo→servo de fixtures são artificiais; não medem
velocidade, centro, raio de curva, frenagem ou direção física.
