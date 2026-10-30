# Portas de estado e projeção optativa

## Contrato implementado: odometry_pose

odometry_pose projeta a **pose existente** de nav_msgs/Odometry para
geometry_msgs/PoseWithCovarianceStamped. Não estima, funde, transforma,
normaliza quaternion, publica TF nem cria covariância/velocidade. Copia exatamente
header e pose com cópia independente. O twist não é consumido nem validado
por esta porta: aceitar a projeção não qualifica o Odometry completo.

Entrada state_odometry e saída state_pose são relativas ao namespace, por
default. parent_frame e child_frame devem ser não vazios e distintos;
max_age_sec precisa ser positivo. Defaults vazios/zero impedem uso sem uma
composição explícita. O nome do corpo deixa de existir no tipo de saída:
**a composição vincula a saída ao child_frame declarado**, que deve coincidir
com o corpo do consumidor. Remapeamento não muda o corpo ou as coordenadas.

A pose deve estar em metros e quaternion de rotação do corpo para o frame pai.
O provedor/perfil deve assegurar essa semântica: números finitos não comprovam
unidades, extrínsecas ou equivalência entre map e odom. O frame recebido
precisa corresponder exatamente ao parent_frame, e o child ao child_frame.
Não há conversão câmera→base nem odom→map neste nó.

## Validação e validade

- Tempo ROS da medida original: stamp positivo e canônico, idade estritamente
  menor que max_age_sec; futuro até future_tolerance_sec inclusive.
- Publicação somente por medida válida estritamente posterior à última aceita.
  Duplicatas/reordenação não movem o cursor. Rejeitar medida inválida não
  consome seu stamp: uma nova medida válida pode recuperar a entrada.
- Posição e quaternion finitos, norma dentro de1e-6 de um
  (quaternion_norm_tolerance). Valores aceitos não são normalizados.
- Covariância6x6 finita, simétrica e sem autovalor menor que-1e-10;
  covariance_tolerance é tolerância absoluta numérica em cada verificação.
  A parte simétrica é usada apenas no teste PSD; a saída conserva todos os
  valores originais. A matriz zero continua zero: isso **não** demonstra
  incerteza nula, qualidade da estimativa ou adequação ao EKF.
- Regressão do relógio ROS observada entre callbacks trava a entrada com
  clock_reset_required, até recriar/reiniciar o nó. Relógio indisponível é
  rejeitado. Para reiniciar/loopar replay com volta de /clock, reinicie
  também o adaptador. Saltos inteiramente sem callbacks não são
  detectados; não existe monitor independente do relógio.
- Silêncio não publica pose nem status novo; não existe timer de republicação.
  Consumidores precisam expirar a medida pelo stamp e detectar silêncio conforme
  sua própria finalidade. Esta fronteira de estado não é watchdog de atuação.

~/status publica std_msgs/String JSON por entrada: aceite/rejeição,
motivo, contagens, frames declarados e latch de relógio. Não é mensagem latched
nem heartbeat. Dados/status usam reliable, volatile e keep-last1. Para fonte
best-effort, a composição deve resolver explicitamente QoS; não existe
descoberta automática de compatibilidade ou de autoridade TF.

## Exemplo exclusivamente sintético

Após build/instalação do pacote, em ambiente de software isolado:

~~~bash
ros2 run ebva_localization odometry_pose --ros-args \
  -r __ns:=/state_fixture \
  -p parent_frame:=fixture_map -p child_frame:=fixture_base \
  -p max_age_sec:=0.2 -p future_tolerance_sec:=0.02
~~~

Esses frames/tempos pertencem à fixture, sem calibração/extrínsecas do
veículo. Nenhum perfil existente inicia o executável.

O consumidor ebva_path_control/waypoint_pp requer
PoseWithCovarianceStamped e é um destino real possível. Só conecte após declarar
o mesmo corpo/frame do caminho e satisfazer a política de validade do controlador.
O PP CSV/Frenet trabalha diretamente com Odometry de semântica própria; não
precisa desta projeção. Não existe obrigação global de usar pose em map.

## Testes

Testes do contrato e do nó verificam cópia de header/pose/covariância,
duplicatas, reordenação, expiração, futuro, frames/corpo, quaternion,
recuperação e regressão de relógio. Casos DDS observam silêncio sem
republicação e a recuperação com medidas novas.

A verificação por callback/replay e o ensaio DDS são complementares.
Nenhum deles mede precisão, deriva, latência do veículo ou extrínsecos.

~~~bash
colcon test --packages-select ebva_localization
colcon test-result --verbose
~~~

Use ROS/dependências preparados e um overlay instalado.
