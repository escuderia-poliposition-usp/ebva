# ebva_supervisor — controle por classificação de pista

## Objetivo

Selecionar leis de controle conforme classificação da pista/posição ou fixar uma lei por perfil. O classificador é externo; o resultado é esforço normalizado com identificador de contrato.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_supervisor/](ebva_supervisor/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [supervisor_node.py](ebva_supervisor/supervisor_node.py): portas ROS e tempo.
- [core.py](ebva_supervisor/core.py): seleção e estado.
- [controllers](ebva_supervisor/controllers): leis de controle.
- [configuration.py](ebva_supervisor/configuration.py): configuração validada.

## Interfaces

`classification/input` (`TrackClassificationStamped`) → `control/autonomy/effort` (`DriveEffort`), com `control/supervisor_status` (`DiagnosticArray`). Tópicos relativos ao namespace. `controller.launch.py params_file:=...` inicia somente `supervisor_node`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md).
- **ROS e bibliotecas externas:** `rclpy`, `rcl_interfaces`, `diagnostic_msgs`, `ament_index_python`, `launch`, `launch_ros`, `python3-yaml`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

O classificador é externo. O adaptador de esforço é outro componente, de ebva_vehicle_control.

## Uso e configuração

### Métodos

| method | Comportamento |
|---|---|
| supervised | Seleciona PI em straight, P de curva em left_curve/right_curve, P de interseção em intersection e recuperação em walls |
| pi_straight | PI de posição, com integral por ciclo e anti-windup existentes |
| p_curve | Direção de curva mais correção proporcional; reduz esforço conforme os limiares existentes |
| p_intersection | Correção proporcional pela posição na interseção |
| wall_recovery | Reproduz a direção do histórico em ordem inversa e inverte o esforço longitudinal; usa o fallback existente quando esgota |
| straight_rule | Publica esforço longitudinal 1 em straight e 0 nas demais classes, com direção 0 |

As leis e seus ganhos iniciais foram preservados. Os controladores operam a
20 Hz; a integral original é por ciclo, portanto mudar a frequência exige
revisão dos ganhos. A recuperação de histórico é uma estratégia algorítmica,
sem alegação de reproduzir fisicamente um trajeto em ré. A regra simples não é
um sistema de inferência fuzzy.

### Interface ROS

O nó supervisor_node tem nome supervisor e usa tópicos relativos ao namespace:

| Tópico | Tipo | Direção |
|---|---|---|
| classification/input | ebva_control_msgs/TrackClassificationStamped | Entrada |
| control/autonomy/effort | ebva_control_msgs/DriveEffort | Saída |
| control/supervisor_status | diagnostic_msgs/DiagnosticArray | Diagnóstico |

A entrada contém header original, valid, track_type, position,
track_probability, track_entropy, position_probability, position_entropy e
curvature_feature. Classes válidas: straight, left_curve, right_curve,
intersection, walls; posições: left, middle, right. Probabilidades devem ser
finitas em [0,1], entropias finitas não negativas e a característica de curvatura
finita. Esta última conserva o escalar do método e sua escala declarada pelo
produtor; não é convertida implicitamente em curvatura geométrica SI.

O frame deve corresponder a expected_frame. O prazo source_timeout_s é
verificado pela idade ROS original e pelo tempo monotônico desde a recepção.
Stamps zerados, futuros, repetidos ou reordenados e mensagens inválidas
interrompem as saídas e limpam integral, histórico e estado de recuperação.
A retomada exige observação válida com stamp novo. Uma nova época de relógio
exige reinício do nó.

DriveEffort mantém o header da classificação. longitudinal e steering estão
em [-1,1]; a direção positiva representa direita, conforme os controladores
originais. O contract identifica a lei efetivamente selecionada:
supervisor_pi_straight_v1, supervisor_p_curve_v1,
supervisor_p_intersection_v1, supervisor_wall_recovery_v1 ou
supervisor_straight_rule_v1. O timer pode calcular novamente dentro do prazo,
mas nunca substitui o stamp da observação pelo horário atual.

### Configuração e composição

Os YAMLs em config são a referência dos defaults e ganhos. input_topic e
output_topic são configuráveis antes da partida; os tópicos exigem reinício
para mudar. Ganhos e method aceitam parâmetros ROS, com validação. A mudança
de método ou frame exige uma classificação nova antes de voltar a publicar.

O launch controller.launch.py recebe params_file e inicia somente o controlador.
Não inicia classificador, simulador, RC, autoridade, backend ou driver.

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=supervisor check_only:=true
ros2 launch ebva_bringup bringup.launch.py profile:=supervisor
~~~

Os perfis supervisor_pi_straight, supervisor_p_curve,
supervisor_p_intersection, supervisor_wall_recovery e supervisor_straight_rule
selecionam as leis fixas por referências aos YAMLs do dono. O perfil padrão
supervisor usa seleção automática.

O adaptador supervisor_effort_adapter pertence a ebva_vehicle_control e
aceita somente mapas software_fixture explícitos. A campanha supervisor_software
exercita controlador, adaptador, autoridade e backend sem dispositivo. Converter
os esforços para um veículo ou simulador exige um adaptador com contrato e
limites próprios; o controlador não publica eRPM, servo ou velocidade física.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_supervisor
source install/local_setup.bash
colcon test --packages-select ebva_supervisor --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

~~~bash
colcon test --packages-select ebva_supervisor
colcon test-result --verbose
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
ros2 run ebva_system_tests supervisor_replay --output /tmp/ebva-supervisor-new
~~~

O destino do replay deve ser novo. Ele utiliza classificação e RC sintéticos,
verifica os modos, prioridade RC, retomada ao neutro válido, perdas e stamps,
e termina com o backend desarmado. Os ganhos desse ensaio são artificiais e
não constituem calibração física.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
