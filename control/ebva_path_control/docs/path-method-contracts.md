# Métodos de trajetória

Os métodos compartilham responsabilidade de controle, mas usam modelos e
referências diferentes. Pure Pursuit no nome não demonstra equivalência entre
as entradas, fórmulas ou saídas.

| Método | Entrada/referência | Saída e particularidades |
| --- | --- | --- |
| waypoint_pp | PoseWithCovarianceStamped e CSV XY ordenado | Ackermann: alvo interno de velocidade por curvatura e direção geométrica; origem/corpo/tempo explícitos |
| waypoint_curvature | PoseWithCovarianceStamped e referência YAML/CSV | Variante com seleção/reaquisição e janela de curvatura próprias; Ackermann estampado |
| csv_frenet_pp | Odometry e CSV s/e_y | Acumula distância absoluta para interpolar erro adiante; ré também avança s |
| lidar_frenet_pp | LaserScan em contrato planar FLU | Medianas laterais e_y=(d_left-d_right)/2; esforço longitudinal fixo e ângulo geométrico |
| waypoint_control | Odometry e CSV x_center/y_center | Modos carrot, stanley e pure_pursuit; esforço longitudinal adimensional e direção geométrica |
| stanley_frenet | Odometry e FrenetLateralErrorStamped | Folgas laterais, referência angular fixa zero e alvo Ackermann; distinto do Stanley waypoint |
| waypoint_preview | Odometry e referência XY | Visualização/diagnóstico, sem contrato de atuação |

## Contratos específicos

- [Waypoint Pure Pursuit](waypoint-pp-contract.md)
- [CSV/Frenet](csv-frenet-contract.md)
- [Esforço e direção geométrica](effort-steering-contract.md)
- [Stanley/Frenet](stanley-frenet-contract.md)

O README do pacote descreve a seleção dos métodos e a interface waypoint_curvature.
Recursos pertencem ao pacote, com paths explícitos. Cada composição declara
referência, corpo/ponto, frames e unidades; o nome do tópico não fornece essas
propriedades.

Odometry→pose é uma projeção optativa, não uma transformação de corpo ou frame.
Um produtor de folgas laterais não fornece automaticamente erro cartesiano
de seguimento. Mapas de esforço→atuação e velocidade/ângulo→atuação pertencem
aos adaptadores, com seus limites e finalidade.

## Composição

Selecione os componentes e configurações pelo bringup. O launch isolado de
um controlador não fornece por si sensores, produtor de estado, referência,
autoridade ou backend. Verifique essas ligações pelo contrato do método.

Mantenha timestamps originais e expiração nos consumidores. Fixtures de
software não qualificam calibração ou comportamento físico.
