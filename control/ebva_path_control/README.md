# ebva_path_control

Controladores distintos de trajetória, com seleção por configuração e perfis ROS.
Os nós não iniciam aquisição, autoridade RC ou driver de atuação.

| Executável | Referência/saída |
|---|---|
| waypoint_pp | Referência XY fechada e pose; AckermannDriveStamped |
| waypoint_curvature | Variante de Pure Pursuit com velocidade por curvatura; AckermannDriveStamped |
| csv_frenet_pp | Referência s/e_y; AckermannDriveStamped |
| stanley_frenet | Folgas laterais estampadas e odometria; AckermannDriveStamped |
| waypoint_control | Carrot, Stanley e PP internos; LongitudinalEffortSteering |
| lidar_frenet_pp | Centralização por medianas do scan; LongitudinalEffortSteering |
| waypoint_preview | Marcadores de diagnóstico; não publica comando |

Ackermann representa velocidade e ângulo geométrico no contrato declarado pelo
método. LongitudinalEffortSteering mantém esforço longitudinal adimensional e
ângulo rad, sem chamar esforço de m/s. As fontes originais de tempo são preservadas;
entradas inválidas/antigas não geram uma nova validade de comando.

Os controladores são selecionados antes do adaptador/autoridade/backend do perfil.
Não há conversão universal de suas grandezas para erpm/servo, e testes sintéticos
não fornecem calibração física. Frames, ponto do modelo, referência e sentido
precisam ser declarados na composição.

## Variante por curvatura

~~~bash
ros2 launch ebva_path_control waypoint_curvature.launch.py
ros2 launch ebva_path_control waypoint_curvature.launch.py params_file:=/caminho/curvatura.yaml
~~~

Recebe PoseWithCovarianceStamped em state/pose e publica
control/autonomy/ackermann. expected_frame (map por padrão) e source_timeout_s
validam a origem; stamps repetidos/reordenados ou expirados não são republicados.
O header de saída é o da pose consumida. reference_file seleciona o YAML da
referência; caminhos relativos do CSV são resolvidos a partir desse YAML.

O default usa data/waypoint_curvature/config_example_map.yaml e seu CSV. Foram
preservadas a seleção de alvo, reacquisição e média de curvatura próprias dessa
variante. lookahead_distance=0.5, velocity_gain=0.5,
wheelbase=0.3147253436255231 e curvature_window=5 conservam os valores do método.
A biblioteca não depende de Gym ou de um renderer.

## Contratos dos demais métodos

- [Waypoint PP](docs/waypoint-pp-contract.md)
- [CSV/Frenet](docs/csv-frenet-contract.md)
- [Stanley/Frenet](docs/stanley-frenet-contract.md)
- [Esforço longitudinal e direção](docs/effort-steering-contract.md)
- [Métodos e recursos](docs/path-method-contracts.md)

waypoint_control mantém métodos distintos e esforço longitudinal 0.2;
lidar_frenet_pp mantém esforço 0.4. Stanley/Frenet conserva seu modelo próprio
(incluindo o alvo assinado de velocidade), distinto do Stanley interno de
waypoint_control. Esses parâmetros não demonstram tracking ou calibração em ré.

O CSV time/e_y/y_center_Ld não é uma referência s/e_y. Fixtures sintéticas
acompanham os testes separadamente e não substituem medidas do veículo.
