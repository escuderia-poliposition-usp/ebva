# Fontes de estado e autoridades TF

## Adaptadores e estimadores

| Função | Contrato e finalidade |
| --- | --- |
| initial_pose | Publica uma pose inicial com covariância configurada; não estima estado |
| pose_republisher | Projeta PoseStamped para PoseWithCovarianceStamped conservando a medida e usando covariância configurada |
| base_link_pose | Consulta TF e publica a pose com timestamp da transformação |
| odometry_pose_tf | Visualização de Odometry como TF diagnóstico; não cria uma estimativa global |
| odometry_pose | Projeta pose/covariância existentes de Odometry, com contrato explícito e sem TF |
| imu_odometry | Prior planar de entrada SI, orientação fornecida, calibração inicial afirmada e integração/ZUPT |
| legacy_imu_prior | Prior alternativo com seu modelo e configuração declarados |
| SLAM, EKF, AMCL e ZED | Provedores selecionados por composição; cada um precisa de entradas e autoridades compatíveis |

[Portas de estado](state-ports.md) descreve a projeção optativa.
O [prior SI](imu-prior-contract.md) declara unidades, calibração e covariâncias.
Covariância configurada não equivale a incerteza medida.

## Composições

O estado local contínuo pode existir sem map. Em composição global, escolha um
dono de map→odom, um de odom→corpo e extrínsecas únicas. SLAM/AMCL fornecem
referência global; EKF, VIO ou prior podem fornecer estado local conforme
seus contratos. Configurações de mapas e estimadores ficam neste pacote.

O corpo da câmera não vira base_link por renomeação. Ao mudar corpo/frame,
transforme valores e covariâncias; confira origem e ponto do modelo consumido.

VESC fornece estado e eco de comandos com a semântica de seu protocolo.
Recepção no host não mede o instante de aquisição no microcontrolador.
Eco de servo não é medição física do ângulo.

A conversão IMU para SI pertence a ebva_sensors. A composição precisa afirmar
as unidades reais antes de convertê-las; não converta dados SI uma segunda vez.

## Observação de TF

A [validação e observação de TF](state-tf-contract.md) separa declarações de
topologia das mensagens observadas. Em Humble Python, a atribuição por amostra
é limitada; endpoints únicos e descoberta do grafo não são autenticação.

Para comparar estimadores simultâneos, use tópicos/frames distintos ou
desabilite TF nos secundários. O monitor não escolhe um estimador vencedor nem
substitui testes de qualidade.

Controladores consomem Odometry, PoseWithCovarianceStamped ou outras portas
conforme o método. O nome de um tópico não comprova corpo, referência ou tempo.
Um adaptador de pose conserva a estimativa; ele não a transforma ou melhora.
