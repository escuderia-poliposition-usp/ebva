# Prior IMU — contrato SI

O executável imu_odometry oferece um prior planar com unidades e validade
explícitas. A alternativa legacy_imu_prior mantém seu modelo próprio; selecionar
uma delas não obriga adotar EKF nem demonstra precisão inercial.

## Entrada e modelo

Entrada relativa imu_si, tipo sensor_msgs/Imu. Saída relativa
imu_prior_odometry, tipo nav_msgs/Odometry. ~/status publica JSON por
entrada, inclusive durante calibração/rejeição. Não há timer, TF, driver,
atuador, republicação ou uso automático por perfil operacional. Dados/status:
RELIABLE, VOLATILE, KEEP_LAST/depth1. A fixture oferece RELIABLE; produtor
BEST_EFFORT não conecta a esta assinatura. A composição precisa resolver QoS
explicitamente, sem adaptação automática ou mudança implícita de assinatura.

A composição deve fornecer explicitamente:

- input_acceleration_units="m/s^2" e input_angular_velocity_units="rad/s".
  Outros valores são recusados, sem inferir g/deg/s de driver/firmware/tópico.
- acceleration_content="specific_force": força específica que inclui a
  contribuição de gravidade no repouso. Entrada já sem gravidade é recusada.
  A calibração estima e subtrai o vetor médio de força específica, incluindo
  gravidade+bias, na referência da orientação.
- input_frame == child_frame: mesma origem/eixos do corpo declarado, com
  aceleração e giro expressos nesse corpo. É afirmação da composição, não
  medição de montagem nem permissão para renomear câmera/IMU para base_link.
- orientation_reference_frame: quaternion fornecido é rotação corpo→referência,
  contínua e com eixos coerentes durante a sessão. Não se infere ENU/NED/
  magnético. Quaternion finito, disponível (orientation_covariance[0] != -1)
  e unitário dentro de1e-6.
- parent_frame distinto do corpo **e de orientation_reference_frame**: plano
  local criado no início, posição zero e
  yaw relativo ao fim da calibração. Integração usa x/y da referência da
  orientação e rotação de -yaw inicial. Para interpretar a saída como plano
  horizontal, a composição assegura z da referência vertical e plano corporal
  apropriado. Nenhuma extrínseca, conversão NED/ENU, lever arm ou medição de
  gravidade é criada.

As três componentes IMU devem estar disponíveis, com vetores e covariâncias de
entrada finitos. Matrizes zero de Imu significam desconhecido e não são usadas
para inferir precisão. O gyro serve para estabilidade/ZUPT, não integra heading.

## Calibração e cálculo preservado

calibration_stationary_asserted=true é obrigatório: operador/composição
afirma repouso na janela inicial. O software **não prova repouso** — velocidade
constante é indistinguível de repouso por estas observações.

A janela exige calib_samples>=2 medidas válidas contíguas, norma de giro
abaixo de calibration_gyro_threshold_rps em todos os eixos, força específica
com norma próxima de expected_specific_force_norm_mps2 dentro de
specific_force_norm_tolerance_mps2, mudança de força na referência menor que
calibration_accel_tolerance_mps2 e mudança de orientação menor que
calibration_orientation_tolerance_rad. Números são escolhas explícitas da
composição, não calibração do carro. Movimento/instabilidade/entrada inválida
reinicia a janela; gap maior que max_integration_dt_sec também. Quaternion q/-q
não representa movimento diferente. Nenhum Odometry sai durante a janela ou
na amostra que a conclui.

Preservam-se orientação fornecida com yaw inicial removido, subtração da média
gravidade+bias, integração semi-implícita planar, ZUPT opcional e cap.
Defaults do método: zupt_enable=true, limiares0.2m/s² e0.05rad/s,
max_speed=4m/s; parâmetros de pesquisa herdados, sem nova medição física.
Velocidade linear é rotacionada do plano local para o corpo usando somente yaw,
como no método original; isso exige plano horizontal/corpo coerentes e **não**
é transformação vetorial3D com roll/pitch arbitrários. Não há gyro→yaw,
fusão, atualização externa de bias ou compensação inercial completa.

## Tempo, silêncio e recuperação

max_age_sec, future_tolerance_sec e max_integration_dt_sec são explícitos.
Stamp original positivo/canônico, idade estritamente menor que max_age,
futuro até tolerância inclusive e cursor estritamente crescente. Duplicatas/
reordenação não renovam validade nem destroem integração válida. Quaternion
nulo/escalado e NaN/Inf não viram identidade/estado não finito.

Entrada inválida interrompe continuidade e zera velocidade, mantendo posição
já integrada. Gap longo também zera velocidade. A primeira medida válida de
recuperação estabelece baseline e **não publica**; a próxima integra somente
o intervalo observado. Não há extrapolação pelo silêncio nem carimbo novo para
estado antigo. Calibração concluída não é refeita após gap; reinicie para
recalibrar se montagem/bias mudar. Regressão observada de relógio ROS trava até
recriar o nó. Silêncio não publica Odometry nem status; consumidores devem
expirar o stamp original.

## Qualidade e covariância

Odometry não tem o sentinel Imu de -1. Portanto a composição fornece
pose_covariance e twist_covariance6x6 explícitas, finitas, simétricas/PSD,
diagonais positivas e covariance_provenance="fixture" ou "research_assumption".
São hipóteses de pesquisa copiadas exatamente, **não incerteza medida/propagada**.
Não há default silencioso de precisão, matriz zero ou calibração.

Status: quality=research_prior, uncertainty_estimated=false,
covariance_propagated=false, stationarity_verified=false. Capacidades:
pose planar relativa e velocidade linear planar. Twist angular é placeholder
zero **não estimado** e não pode alimentar máscara/consumidor que o exija.
Não se qualificam roll/pitch, vertical, deriva, precisão ou latência veicular.
Não se fabricou gyro como derivada do heading.

## Fixtures e testes

config/imu_prior/si_fixture.json contém escolhas sintéticas de corpo
co-localizado, força específica, tolerâncias e covariâncias. Não é configuração
do veículo. O fixture de regressão em test/data conserva as saídas numéricas
necessárias para verificar o método.

Os testes exercitam calibração, duplicatas, quaternion inválido, corpo errado,
expiração, NaN, silêncio, recuperação, frames/stamps e covariâncias.
A campanha de software grava uma bag sintética e utiliza ros2 bag play:

~~~bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=142   ros2 run ebva_system_tests imu_prior_replay --output /tmp/imu-prior-campaign-new
~~~

Escolha domínio livre e destino novo. A campanha não inicia TF, sensores ou
atuação. Dados, logs e resultados são artefatos locais, fora dos fontes.

Antes de uma composição física, confirme unidades do fornecedor/firmware,
referência da orientação, frame/origem/extrínseca e repouso real. Defina
incerteza e consumidores compatíveis. Dados sintéticos não medem essas
propriedades.
