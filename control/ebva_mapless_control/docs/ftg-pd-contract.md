# FTG/PD: leis e contrato de duas observações

ftg_pd utiliza FtgPdCore sem ROS para a busca de gap e as leis de controle;
o wrapper aplica o contrato temporal e publica FtgPdCommand. scan_direct e
scan_pd são implementações distintas, descritas no
[README do pacote](../README.md).

## Diferenças entre métodos

| Executável | Longitudinal | Saída lateral efetiva |
|---|---|---|
| scan_direct | TTC 10, teto 0,24, subida 0,12 por amostra | Ângulo direto |
| scan_pd | TTC 50, teto 0,12, subida 0,02 por amostra | Ângulo direto; o trecho PD não é executado |
| ftg_pd | TTC 30, teto 0,12, subida 0,02 por amostra | PD sobre referência e feedback numéricos |

Nomes ou comentários não tornam as saídas equivalentes. Com referência 0,2
e feedback 0,1, o PD inicial produz 0,05556; os métodos diretos publicam 0,2.
O adaptador ftg_pd_adapter não aceita saídas de outros métodos como se tivessem
o mesmo contrato.

## Lei longitudinal

A busca FTG fornece distância D e ângulo A:

~~~text
q = (D/30)*(1-|graus(A)|/(2,5*60))
s[k] = min(q, s[k-1]+0,02)
throttle[k] = min(s[k],0,12)
safeCurveExit = 1
~~~

O estado interno s é anterior à saturação publicada. Não há feedback de
velocidade; apesar da expressão distância/tempo, throttle é a grandeza numérica
do modelo e não uma velocidade física comprovada. No cenário reto D=3, o regime
é 0,1. Curvas reduzem o valor; quedas não usam o mesmo limite da subida.

## Referência e PD

A referência angular é limitada a ±60° e sua variação a 20° por callback.
O PD usa Kp=1, Td=0,2, N=10 e Ts=1/40, com derivada filtrada sobre a medida:

~~~text
ud[k] = (4/9)*ud[k-1] - (4/9)*(medida[k]-medida[k-1])
u[k] = referencia_limitada[k] - medida[k] + ud[k]
~~~

Não existe saturação final no PD. Referência -pi/3 e feedback 0,8 produzem
aproximadamente -2,2027531. Essa saída é esforço PD; não é ângulo Ackermann
ou esforço necessariamente em [-1,1]. A limitação de servo pertence ao mapa
posterior explícito.

Parâmetros do modelo: carDiameter=0,8, tolMaskDisparities=0,01,
disparityTreshold=0,05 e SafeLateralDistance=0,1. Esses valores não são
dimensões ou distâncias medidas no veículo atual. O limite ±60° é interno
à referência e não qualifica o curso mecânico.

Ts e rampas são fixos por amostra. Mudar frequência ou jitter exige avaliação
própria. A geometria do gap, o mascaramento e a regra lateral mantêm suas
leis; isso não demonstra eficácia física de prevenção de colisão.
O mascaramento à esquerda limita sua fronteira inferior a zero, impedindo
que índices negativos alterem o final do vetor.

## Duas observações e par atômico

O controlador exige scan e feedback com aquisição explícita. O perfil de
software usa JointState com joint virtual_steering, posição em radianos
sintéticos, frame declarado e propósito software_fixture_angle_rad.
Isso não atribui escala ao AutoDRIVE nem calibra um sensor real.
Recepção de um Float32 sem stamp não é tratada como aquisição da medida.

O scan precisa de metadados finitos/coerentes, incremento positivo, cobertura
de pelo menos ±95° e raios positivos finitos dentro de range_min/range_max.
A fronteira rejeita NaN/inf em vez de substituí-los por range_max.
Retorno ausente de um LiDAR real pode exigir uma política de entrada própria.

FtgPdCommand reúne os dois números e conserva header do scan e feedback_header
da observação usada; contract=ftg_pd_sample_v1. O par requer:

| Condição | Limite do perfil de software |
|---|---|
| Idade de cada medida | <200 ms |
| Tolerância futura | 20 ms |
| Diferença entre stamps | <=80 ms |

Duplicatas/reordenação não renovam observações. Saída não representável em
Float32 é rejeitada sem encerrar o nó ou renovar comando. Após lacuna válida
>=200 ms, o núcleo reinicia memória, PD e rampas. A política vale para a
fronteira; as leis das sequências contínuas permanecem as mesmas.

MeasurementLease, em ebva_control_contracts, usa idade ROS original e prazo
monotônico por fonte. Clock pausado não prolonga dados. Uma época anterior
de replay exige reinício ou reset explícito do consumidor.

## Adaptação e autoridade

ftg_pd_adapter exige o contrato e propósito software_fixture. O feedback tem
lease independente: scans novos não renovam medida antiga. ActuatorCommand
recebe o header da entrada mais antiga, como limite conservador de validade.
Esse stamp não é um identificador único de ciclo; a mensagem FTG conserva
as duas procedências.

Os frames são iguais por construção do teste, sem transformação implícita.
O mapa sintético usa 1000 eRPM/unidade de throttle, -0,1 servo/unidade de esforço
PD e centro 0,5, com limites ±600 eRPM e servo 0,4–0,6.
Isso não é calibração física. O adaptador não usa conversão Ackermann nem
impõe ao PD o domínio normalizado do FRM.

A política compartilhada StampedFixtureAdapter verifica tempo/frame e mantém
os mapas específicos. A seleção RC e o backend pertencem a
[ebva_vehicle_control](../../ebva_vehicle_control/README.md). RC seleciona o
par manual completo; ao neutro, autonomia retoma somente com entradas válidas.

Silêncio de scan ou feedback, ou dados velhos/inválidos, expiram o par na
cadeia de autoridade/backend. Não há heartbeat de comando que renove a idade
do controlador ou adaptador; diagnósticos também não renovam a validade.

## Verificação e composição

~~~bash
colcon test --packages-select ebva_mapless_control ebva_vehicle_control
colcon test-result --verbose
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
ros2 run ebva_system_tests ftg_pd_replay --output /tmp/ebva-ftg-pd-new
~~~

O destino do replay deve ser novo. O perfil e a campanha de software pertencem
a ebva_system_tests e utilizam somente entradas sintéticas.
Consulte [testes](../../../docs/testing.md),
[interfaces](../../../docs/interfaces.md) e
[autoridade](../../../docs/command-authority.md).

Uma composição física precisa declarar feedback real, frames/extrínsecos,
amostragem, sinais de direção/propulsão e mapas de atuadores. Os testes de
software não verificam essas grandezas, RF ou watchdog de firmware.
