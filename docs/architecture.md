# Arquitetura

## Pacotes por responsabilidade

As fronteiras dos pacotes seguem função e dependências, sem exigir um Git ou
pacote por variante de algoritmo. Métodos distintos permanecem identificados
e podem ser combinados por perfis.

ebva_bringup carrega perfis YAML e resolve recursos instalados.
Defaults e validações pertencem aos donos: veículo em ebva_vehicle_control,
mapas na localização, modelo de placas no detector. O perfil referencia esses
recursos sem duplicá-los.

| Função | Pacotes |
| --- | --- |
| Aquisição, autoridade e atuação | ebva_vehicle_control |
| Contratos/mensagens | ebva_control_contracts, ebva_control_msgs, lane_msgs |
| Trajetória | ebva_path_control |
| Faixas | ebva_lane_detection, ebva_lane_hough, ebva_lane_keeping |
| Mapless/FRM/CBF/supervisão | ebva_mapless_control, ebva_frm_control, ebva_cbf_control, ebva_supervisor |
| Sensores/unidades | ebva_sensors |
| Placas/profundidade/nuvens | ebva_sign_detection, ebva_sign_depth, ebva_pointcloud_fusion |
| Estado, TF, estimadores e mapas | ebva_localization |
| Navegação/referências | ebva_navigation, ebva_reference_tools |
| Gravação/observação | ebva_recording, ebva_diagnostics, ebva_analysis |
| Simulação | ebva_autodrive, ebva_simulation |
| Calibração/testes | ebva_calibration, ebva_system_tests |

## Dados e controle

~~~mermaid
flowchart LR
  Sensores --> Estado[Percepção / estado]
  Referencias[Referências] --> Controle[Controlador escolhido]
  Estado --> Controle
  Controle --> Adaptador[Adaptador do contrato]
  RC[RC adquirido] --> Autoridade
  Adaptador --> Autoridade
  Autoridade --> Backend[Backend e limites]
  Backend --> Driver[Driver VESC]
~~~

A atuação é uma composição explícita. O controlador fornece um resultado
com significado próprio; seu adaptador faz a conversão declarada para o
consumidor. Esforço adimensional não é velocidade ou aceleração.

A autoridade RC escolhe o par inteiro motor/direção. A composição de ensaio
direto tem outro contrato, descrito em [autoridade](command-authority.md).

## Estado e percepção

Odometria contínua em `odom` e referência global corrigível em `map` têm
papéis diferentes. Cada composição escolhe autoridades TF e declara corpo e
ponto do modelo. Mapless não exige localização global artificialmente.

Percepção 2D, profundidade, faixas e nuvens mantêm contratos próprios.
Métodos Hough/segmentação podem oferecer geometrias diferentes; unidade,
origem e validade acompanham a seleção.

## Simulação e testes

ebva_autodrive liga ROS ao simulador AutoDRIVE e mantém sua interface de
teleop. Cálculo de trajetória pertence ao controle, permitindo trocar
simulador e controlador separadamente.

ebva_simulation fornece fontes sintéticas e modelos. ebva_system_tests contém
campanhas e verificações. A aplicação não importa esses testes nem scripts
do ambiente de desenvolvimento para executar.
