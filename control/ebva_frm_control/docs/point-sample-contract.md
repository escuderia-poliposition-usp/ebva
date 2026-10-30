# FRM pontual: lei, geometria e fronteira ROS

FrmPointCore contém a lei sem I/O; point_sample adapta LaserScan para uma
mensagem atômica DriveEffort. A implementação scan_average é um método distinto
e mantém sua própria seleção. Consulte o [README do pacote](../README.md).

## Lei longitudinal

Definindo d[k]=min(range[540],10), a saída é:

~~~text
u_long[k] = clip(0,03*d[k] + 0,01*(d[k]-d[k-1]), -1, +1)
d[-1] = 0
~~~

Não há divisão por dt, realimentação de velocidade ou integração de aceleração.
Com d=5, a primeira amostra produz 0,20 e as seguintes 0,15.
d=10 seguido de 0,01 produz aproximadamente -0,0996 em uma amostra.
Para d em [0,10], o intervalo efetivo da lei é [-0,1;0,4], dentro da saturação
explícita ±1.

A saída é esforço adimensional; valor negativo não demonstra frenagem ou
marcha à ré desejada. Mudar a frequência de amostragem altera o comportamento
no tempo físico e exige avaliação própria.

## Lei lateral

~~~text
e1[k] = clip(range[900]-range[180], -10, +10)
e2[k] = clip(range[720]-range[360], -10, +10)
u_lat[k] = clip(0,6*e1[k] + 4*(e1[k]-e1[k-1])
               + 0,02*e2[k] + 0,3*(e2[k]-e2[k-1]), -1, +1)
~~~

Duas médias exponenciais com alpha=0,2 permanecem no cálculo, com ganhos
K5/K6 iguais a zero. O valor lateral é um esforço adimensional assinado e
limitado, com memória por amostra. Não é yaw rate ou ângulo geométrico:
a lei não declara dt ou calibração angular. Sequências espelhadas preservam
sinais opostos; variações bruscas podem saturar.

Os testes de regressão numérica verificam pares e memória do núcleo para
sequências contínuas; os testes da fronteira verificam sua recuperação.

## Geometria e entrada

Os cinco índices são fixos. O wrapper exige:

| Propriedade | Contrato |
|---|---|
| angle_min | -135° |
| angle_increment | 0,25° |
| angle_max | Coerente com a quantidade de raios |
| Quantidade | Pelo menos 901 raios |
| Raios consumidos | 180, 360, 540, 720 e 900 |

Nessa geometria, 540 representa 0°; 180/900 representam -90°/+90°;
360/720 representam -45°/+45°. O wrapper não remapeia os índices para outro
LiDAR ou altera a fórmula para compensar metadados incompatíveis.

Os cinco valores consumidos devem ser finitos e estar em range_min/range_max.
Raios não consumidos não são preenchidos ou filtrados pela adaptação.
O frame esperado é obrigatório e verificado. O frame fixture_lidar_flu dos
testes é sintético e não mede a orientação ou extrínsecos do sensor real.

## Tempo, invalidez e recuperação

Scan curto, metadados incompatíveis ou raios consumidos inválidos são rejeitados
antes de alterar o núcleo. Stamp zero, velho, futuro além da tolerância de
20 ms, duplicado ou reordenado não renova a saída. No perfil de software,
a idade máxima é 200 ms. O header original é copiado sem transformar a
recepção em nova aquisição.

Após intervalo sem entrada válida >=200 ms, a primeira amostra reinicia a
memória do núcleo. Essa política da fronteira evita diferenciar a lei através
de uma lacuna. Uma época de replay anterior exige reiniciar os nós.
Diagnóstico periódico não renova comandos. Este método não utiliza latch
de faixas.

## Saída atômica e autoridade

DriveEffort contém header original, contract=frm_point_sample_v1 e os
Float32 longitudinal/steering no mesmo par. A atomicidade nasce no produtor,
sem emparelhar campos por proximidade temporal.

O nó publica somente control/autonomy/effort. Não publica os campos em tópicos
independentes nem comandos VESC. A conversão e a seleção manual/autônoma
pertencem a [ebva_vehicle_control](../../ebva_vehicle_control/README.md).
Intervenção RC seleciona o par manual completo; a retomada ao neutro depende
de entradas válidas.

Entrada rejeitada ou silenciosa expira na cadeia de autoridade/backend.
O controlador não envia heartbeat que prolongue o último esforço.

## Adaptação e testes

frm_effort_adapter aceita somente o contrato frm_point_sample_v1 e propósito
software_fixture, com frame, identificador e ganhos explícitos. A configuração
de teste usa 100 eRPM/esforço, -0,1 servo/esforço e centro 0,5.
Esses valores são artificiais; não convertem o longitudinal em m/s nem
constituem calibração do veículo.

A política ROS de stamp/frame/expiração é compartilhada em ebva_vehicle_control,
mas cada adaptador conserva as unidades e o domínio de seu contrato.

~~~bash
colcon test --packages-select ebva_frm_control ebva_vehicle_control
colcon test-result --verbose
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
ros2 run ebva_system_tests frm_replay --output /tmp/ebva-frm-new
~~~

O destino do replay deve ser novo. O perfil de software e as verificações
pertencem a ebva_system_tests. Ele exercita valores sintéticos e não abre
dispositivos. Consulte [testes gerais](../../../docs/testing.md) e
[interfaces](../../../docs/interfaces.md).

A variante por média de dez raios, FTG/PD e CBF continuam métodos distintos.
Uma composição física exige geometria e amostragem conhecidas, resposta do
esforço, sinal, calibração, modo VESC para esforço negativo e comportamento
de propulsão. Testes sintéticos não qualificam esses aspectos, frenagem, RF
ou watchdog de firmware. Veja a
[referência de autoridade](../../../docs/command-authority.md).
