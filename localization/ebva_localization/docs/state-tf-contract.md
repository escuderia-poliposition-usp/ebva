# Contrato de composição estado/TF e observação optativa

## Duas verificações distintas

validate_profile verifica **declarações** de fontes, consumidores e autoridades.
O monitor state_tf_audit verifica **observações TF/DDS** das arestas declaradas:
endpoint, pai/filho, tipo, quaternion e tempo. Nenhuma etapa mede extrínsecas,
deriva/precisão, autentica nós ou qualifica veículo. O monitor publica somente
status; não bloqueia, transforma, remove ou republica TF/estado/comandos.

Portas de estado são verificadas somente pela declaração nesta ferramenta.
Os respectivos produtores/consumidores validam mensagens e expiração em runtime;
número finito ou tipo ROS não comprova unidade/corpo correto.

## API pura para composições

~~~python
from ebva_localization.tf_contract import validate_profile
validated = validate_profile(declaration)
~~~

Recebe dict, devolve cópia JSON-ready com declaration_valid=true e
tf_validation_scope=declared_edges (lista não vazia) ou not_requested.
Levanta ValueError por declaração inválida/incompatível; não muda o dict
original nem importa ferramentas da integração. Recursos de exemplo instalados
ficam em config/state_tf, todos exclusivamente sintéticos.

Campos do perfil:

| Campo | Contrato |
| --- | --- |
| profile_id | Nome explícito |
| local_frame, global_frame | Nomes distintos ou null; ao menos um existe |
| body_frame | Corpo escolhido, distinto das referências |
| require_tf | Booleano; false permite ligar mensagens coerentes sem fabricar TF |
| static_lifetime | publisher_present: estáticos só são observados válidos com o endpoint original presente |
| future_tolerance_sec | Tolerância finita não negativa, menor que cada lease dinâmico |
| edges | Lista parent/child/publisher/kind; publisher é FQN, kind dynamic ou static; dynamic exige max_age_sec positivo |
| state_ports | Lista de contratos do estado ofertado |
| consumers | Lista de requisitos e ligação à porta ofertada |

Cada child TF possui um único pai/dono declarado; ciclos/autoarestas são
rejeitados. require_tf=true exige caminho local→corpo e global→local quando
ambos existem. Perfil exclusivamente global (local_frame=null) exige
global→corpo. Intermediários/extrínsecas são permitidos, sem transformações
inventadas. Perfil local não recebe obrigação de map. Alternativas de
estimadores usam declarações próprias; a lista não ativa dois donos na mesma
aresta. Quando require_tf=false, a ausência de TF não invalida mensagens com
pai/corpo coerentes; arestas ainda declaradas são verificadas estruturalmente.

## Portas de estado e consumidores

Cada state_port exige name, publisher (FQN), topic, type, role, frame, body,
model_point, stamp=measurement, max_age_sec positivo e provides não vazio.
role local/global vincula frame ao respectivo local_frame/global_frame.
body precisa coincidir com body_frame do perfil. type aceito:
nav_msgs/Odometry ou geometry_msgs/PoseWithCovarianceStamped.

Capabilities permitidas:

| Capability | Significado declarado |
| --- | --- |
| pose | Pose do corpo/ponto no frame pai, metros/quaternion ROS |
| pose_covariance | Covariância fornecida da pose; não é alegação de precisão |
| longitudinal_velocity, lateral_velocity, vertical_velocity | twist.linear.x/y/z, m/s no corpo/child |
| roll_rate, pitch_rate, yaw_rate | twist.angular.x/y/z, rad/s no corpo/child |
| twist_covariance | Covariância fornecida do twist, sem estimativa inventada |

Odometry exige twist_frame==body. Pose só oferece pose/pose_covariance e não
aceita twist_frame. O corpo de Pose não existe no payload: sua associação é
afirmação explícita da composição, sem prova física por este validador.
Uma transformação TF existente não transforma automaticamente uma mensagem.

Cada consumidor exige name, port, type, frame, body, model_point, max_age_sec
e needs não vazio. Tipo/pai/corpo/ponto devem coincidir exatamente; needs deve
ser subconjunto de provides. A porta não pode declarar idade maior que a
aceita pelo consumidor. Esses valores descrevem políticas das fronteiras,
não uma detecção viva de qualidade ou garantia de taxa/sincronização.

CBF precisa longitudinal_velocity em child_frame_id no ponto do modelo;
não é derivada de posição. Waypoint PP consome pose do mesmo corpo/frame/ponto
da referência; não exige velocidade que não usa. Referências globais podem
corrigir/descontinuar pose e odometria local pode derivar: compatibilidade de
nomes não demonstra continuidade/adequação dinâmica de um provedor.

## Monitor ROS optativo

~~~bash
ros2 run ebva_localization state_tf_audit --ros-args \
  -p profile_path:=/caminho/absoluto/declaracao.json
~~~

profile_path é obrigatório. tf_topic=/tf e tf_static_topic=/tf_static são
distintos e remapeáveis por parâmetros; audit_period_sec default0.05, positivo
até1s. Inscrições reliable keep-last100, volatile para dinâmico e
transient_local para estático. Timer usa STEADY_TIME: silêncio vence mesmo com
relógio ROS pausado. Nenhum perfil ativo inicia o monitor por existir no pacote.

TFMessage não carrega identidade por aresta. A imagem Humble atual não
entrega MessageInfo ao callback Python: Executor._take_subscription conserva
somente msg_info[0], e a metadata nativa inspecionada contém apenas
source_timestamp/received_timestamp, sem publisher_gid. Os dois pilotos DDS
reproduziram a incompatibilidade, sem xfail ou alegação de atribuição inexistente.

O monitor usa callback de um argumento e **inferência conservadora** somente
quando existe exatamente um endpoint TOTAL no tópico/kind. A descoberta cruza
FQN/GID do endpoint com a declaração, mas não comprova a origem de cada amostra.
Outro publisher, mesmo com FQN diferente ou aparentemente irrelevante, torna
o stream ambíguo e impede observations_valid. Dois endpoints com o mesmo FQN
também são ambíguos. Endpoint que muda durante a espera invalida os dados
anteriores; uma mensagem cuja origem não podia ser inferida na recepção não é
atribuída depois a um publisher que surgiu.

Status explicita attribution=unique_endpoint_inference,
per_message_gid_available=false, identity_per_sample=unavailable e
sample_authority_verified=false. observations_scope descreve apenas conteúdo/
tempo TF e inferência por endpoint único; não existe autenticação ou isolamento.
O núcleo puro TfAudit aceita GID fornecido pelo chamador e testa conflitos, mas
isso não significa que o callback Humble recebeu GID da amostra.

Por isso, autoridades local/global distintas em um tópico /tf compartilhado
não são qualificadas por este monitor Python. A declaração pode ser válida
enquanto a observação permanece ambiguous_stream. A fixture global DDS usa um
único broadcaster sintético para as duas arestas dinâmicas, mantendo o estático
em outro tópico. Os recursos global_fixture.json (autoridades distintas) e
global_single_broadcaster_fixture.json deixam essa diferença explícita.
Verificação futura de origem por amostra em streams compartilhados exige
observador com metadata adequada (por exemplo rclcpp), em etapa própria;
não se alterou rclpy nem o empacotamento para ocultar a limitação.

Antes de todos os endpoints selecionados estarem únicos/presentes, dados
aguardam em fila limitada64 (necessária para primeiro TF estático latched).
Overflow é rejeição observável. A inferência exige endpoint único já conhecido
na recepção; se ainda não havia identidade disponível, a observação é rejeitada
após readiness e requer nova publicação, sem atribuir origem retroativamente.
Dados dinâmicos enfileirados conservam o limite
monotônico desde a recepção original; descoberta tardia não os rejuvenesce.
Endpoint desconhecido/mudado, tópico estático/dinâmico errado, aresta não
declarada, números não finitos, quaternion fora da norma1±1e-6 e tempo inválido
são rejeitados. Valores aceitos não são normalizados nem reescritos.

Dinâmicos exigem stamp canônico positivo, prazo ROS original e limite
monotônico; duplicatas/reordenação não renovam prazo. Estáticos aceitam stamp
zero e não expiram por idade, mas deixam de ser válidos se o GID original
desaparece/troca. Um buffer tf2 pode continuar contendo esse transform após
desaparecimento: isso não satisfaz a política explícita publisher_present.
A observação do grafo tem latência de descoberta; não prova detecção instantânea.

Status JSON em ~/status separa declaration_valid, discovery_ready,
observations_valid, issues, rejections e clock_reset_required.
Rejeições de conteúdo/identidade são contadas e mantêm observations_valid=false
até reiniciar o monitor; renovação de TF após simples expiração pode recuperar.
Regressão do relógio ROS trava clock_reset_required até reiniciar; timer
monotônico continua observando. Readiness é apenas descoberta dos endpoints TF;
não inclui publishers das portas de estado nem prova que já enviaram medidas.

Campos de limite permanecem explícitos: state_validation_scope=declarations_only,
authority_authentication=false e physical_qualification=false. Comparar
algoritmos simultaneamente exige frames/tópicos próprios ou TF desligado nos
secundários; o monitor não escolhe silenciosamente um vencedor.

## Testes

Testes puros cobrem local sem map, global+local e global direto, mensagem sem TF,
incompatibilidades de corpo/ponto/tipo/capabilities/idade, ciclos/múltiplos donos,
GIDs duplicados/mudados, stamps, expiração, fila de descoberta e relógio pausado/
regressão. Duas fixtures DDS/tf2 exercitam TF estático antes da descoberta,
transformação composta conhecida, silêncio e recuperação, dois endpoints com
o mesmo FQN e perda do publisher estático apesar do cache tf2. Geometria dessas
fixtures é artificial, sem inferir montagem do veículo.

O monitor não repara fontes de estado. Cada produtor continua responsável por
finitude, orientação, unidades, frame, covariâncias e validade segundo seu
contrato. O diagnóstico de topologia não demonstra precisão física.
