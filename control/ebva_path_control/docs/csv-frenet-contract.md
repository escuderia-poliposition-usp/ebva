# CSV/Frenet: referência escalar e alvo Ackermann

Modelo de avanço escalar sobre uma referência s/e_y. Conversão física e seguimento
no veículo exigem configuração e avaliação próprias.

## Semântica do modelo

O recurso tem colunas nomeadas s[m],e_y[m], amostras finitas, s estritamente
crescente desde zero e comprimento final>=1,2m. Ordem crescente significa o
avanço no recurso. A composição afirma que a primeira odometria aceita
corresponde a s=0. O progresso acumula **distância absoluta** entre posições XY;
ré também avança s. O algoritmo não projeta a pose numa centerline cartesiana,
não usa yaw para corrigir orientação e não calcula erro transversal à referência
cartesiana. e_y é a referência escalar interpolada, não uma observação atual
calculada da posição do veículo.

Preservam-se entre-eixos L=0,26m, Ld=1,2m, kappa=2*e_y/Ld²,
delta=clip(atan(L*kappa),±0,4rad) e
v_cmd=clip(0,5/(1+0,8*abs(kappa)),0,4,0,5). Esses parâmetros são os do modelo
selecionado e não uma medição de entre-eixos/calibração do veículo.
O [fixture numérico](../test/data/csv-frenet-golden.json) verifica os pares e
estados de progresso, incluindo passagem pelo fim do recurso.

O progresso e o lookahead mantêm o wrap simples original, inclusive valor
exatamente no fim do recurso. Para não interpolar silenciosamente fora dessa
hipótese, o novo contrato rejeita recursos menores que Ld e deslocamentos entre
amostras maiores que o comprimento do recurso. Saltos/reset de odometria,
voltas perdidas e alinhamento inicial arbitrário não são suportados. Não foi
inventado um limite dinâmico de velocidade física para aceitar/rejeitar passos.

## Interface Ackermann

O nó anterior publicava throttle=v_cmd*0,5, anotado no código como teste, e
delta em dois Float32 sem header. A fronteira nova publica **o alvo interno
v_cmd** em AckermannDriveStamped.speed[m/s], sob contrato SI afirmado pela
composição, e delta em steering_angle[rad], positivo à esquerda. O fator0,5
foi retirado da borda; isso não demonstra uma relação medida entre throttle e
velocidade. Não há conversão física para eRPM/servo no pacote.

Uma entrada aceita produz um único par atômico. Não há timer/heartbeat para
renovar saídas a partir de odometria velha. A subscrição LaserScan vazia foi
retirada: ela nunca contribuía para as saídas nem detectava obstáculos.
A nova fronteira não passa a oferecer detecção de obstáculos por causa disso.

O stamp de saída é copiado exatamente da odometria. O frame de saída é o corpo
no ponto do modelo, declarado pela composição. Isso difere do header.frame_id
da odometria, que descreve **o referencial da pose**: o comando é relativo ao
corpo, não uma pose mundial renomeada. child_frame_id deve representar o eixo
traseiro do modelo ou uma fixture explicitamente coincidente. Não transformar
ou renomear base_link para fingir coincidência física; a composição fornece
localização no ponto correto e valida sua relação com TF/caminho.

## Parâmetros e tópicos

| Parâmetro obrigatório | Valor/afirmação |
| --- | --- |
| csv_path | Recurso s,e_y SI adequado à finalidade acima |
| reference_frame_id | Pai da pose, comparado ao header da odometria |
| body_frame_id | Corpo/ponto, comparado ao child_frame_id e usado no comando |
| model_point | rear_axle ou software_fixture_rear_axle; sem default |
| reference_contract | closed_progress_si_v1 |
| progress_origin | first_accepted_odometry_s0 |
| progress_direction | path_order |

observation_timeout=0,2s e future_slack=0,02s são política explícita do cenário
de software. odometry_topic=odometry e command_topic=ackermann_command são
relativos e podem ser ligados/remapeados pelo perfil. QoS reliable/volatile,
keep_last1, nos dois lados. Parâmetros são lidos na inicialização.

A aquisição deve fornecer posição/quaternion unitário e componentes de
velocidade consumidas finitas, pose SI com mesmo pai e corpo estáveis. Covariância
não entra nesta lei e não foi convertida em garantia de qualidade. Twist é lido
para manter o estado do modelo, mas não altera a lei v_cmd atual.

MeasurementLease compartilhado verifica idade original, tolerância futura,
frame, duplicatas e reordenação; antes da conversão, Time exige sec>=0 e
0<=nanosec<1e9. Time malformado produz invalid_stamp sem renovar estado/lease.
O prazo monotônico impede renovar a mesma
amostra com relógio ROS parado. Rejeições não atualizam estado nem prazo.
Valores finitos cujo hypot não é representável também são rejeitados antes
atualizar o lease. Ao silenciar, o produtor simplesmente deixa de publicar;
**a expiração e parada efetiva dependem do consumidor/autoridade/backend**.

Após lacuna, uma nova amostra válida mantém s e integra deslocamento desde a
última posição aceita. Não reinicia s=0, pois isso realinharia o recurso sem
uma nova origem. A distância reta subestima movimento curvo perdido; amostragem
suficiente e ausência de saltos são condições do perfil. Uma recuperação não
qualifica reconstrução de trajetória perdida.

DiagnosticArray no tópico relativo control/path_status é publicado por entrada,
sem renovar comando: header original, status.name=csv_frenet_pp, message=accepted
ou motivo de rejeição, progress_m/body_frame/source_stamp_s e campos brutos
source_stamp_sec/source_stamp_nanosec. No Time malformado, source_stamp_s=invalid. Não existe heartbeat
de estado. Mensagens futuras/antigas/frames errados não recebem um header novo
que esconda sua procedência.

## Execução de software

Com ROS Humble e os dois pacotes instalados, o recurso/YAML de fixture acompanha
ebva_path_control. Um exemplo de composição do nó, sem drivers:

```bash
ros2 launch ebva_path_control select.launch.py method:=csv_frenet_pp \
  csv_path:=/caminho/absoluto/reference.csv \
  reference_frame_id:=fixture_map body_frame_id:=fixture_rear_axle_flu \
  model_point:=software_fixture_rear_axle reference_contract:=closed_progress_si_v1 \
  progress_origin:=first_accepted_odometry_s0 progress_direction:=path_order
```

Essa entrada inicia somente o controlador, sem entradas sintéticas, adaptador,
aquisição RC ou backend. config/csv_frenet_software.yaml e
 data/csv_frenet_fixture/reference.csv são fixtures opcionais; csv_path precisa
ser fornecido explicitamente. O perfil integrado é responsabilidade do bringup.

Em workspace isolado, selecionar este pacote e ebva_control_contracts por
base-paths explícitos no build **e no test**. Com --merge-install no build, usar
a mesma opção no test; executar source install/setup.bash antes dos testes.
A seleção de build pertence à composição e suas dependências. Os testes usam apenas
fontes/recursos e dependências ROS/Python, sem integração/bancada/ambiente.

A suíte do pacote cobre23 pares originais; leis e sinais; parâmetros obrigatórios;
referência inválida; NaN/Inf/quaternion; corpo/pai; stamp antigo/futuro/duplicado;
clock parado; salto de progresso; lacuna/recuperação; e exercício DDS isolado
entrada→comando/diagnóstico com preservação de header e silêncio. Esse exercício
é replay sintético em DDS, não ensaio físico ou rosbag real do veículo.

Os outros métodos e recursos têm semânticas próprias;
[seus contratos e pendências](path-method-contracts.md) são independentes.


O encerramento destrói o nó e usa rclpy.try_shutdown(), API Humble que aceita
contexto já desligado pelo SIGINT. Testes dirigidos reproduzem esse estado e
confirmam que falhas arbitrárias ainda propagam após a limpeza.
