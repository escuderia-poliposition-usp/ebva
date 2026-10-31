# EBVA — Escuderia Brasileira de Veículos Autônomos

O EBVA é a plataforma ROS 2 do carro F1TENTH da **Escuderia Poliposition USP**.
Este monorepo reúne 27 pacotes próprios: sensores, percepção, localização,
referências, controladores, interface do veículo, gravação, análise e testes.
Seu objetivo é desenvolver e comparar métodos de condução no mesmo carro,
com interfaces explícitas e composições reproduzíveis.

As alternativas de controle e localização são selecionadas por **perfis de
bringup**. Cada pacote mantém código, dependências, parâmetros e testes;
o perfil conecta as peças. Instalar um pacote não inicia seus nós.

[Wiki: aprender a usar](https://github.com/escuderia-poliposition-usp/ebva/wiki) ·
[Referência técnica](docs/README.md) · [Contribuir](CONTRIBUTING.md)

## Ambiente e requisitos

| Contexto | Preparação |
| --- | --- |
| Carro / Jetson ARM64 | Ubuntu 22.04, ROS 2 Humble e Python 3.10 nativos; SDKs/drivers compatíveis com o alvo |
| PC Linux | Base ROS para desenvolvimento/testes; recursos dependem dos pacotes selecionados |
| WSL x86_64 | Docker de desenvolvimento mantido separadamente; adaptação ao monorepo pendente |
| Simulação | Fontes sintéticas no EBVA; AutoDRIVE exige simulador externo; integração Gym é futura |

Prepare Git, compilador/CMake, colcon, rosdep e vcstool. As dependências ROS
estão nos package.xml; fornecedores têm versões em [dependencies.repos](dependencies.repos).
ZED requer SDK/GPU; YOLO requer PyTorch/Ultralytics e checkpoint; localização
precisa dos mapas correspondentes. Esses recursos não vêm com o clone.

O produto pode ser construído com dependências ROS/Python sem checkout da
bancada ou scripts de outro repositório. Não copie venvs, builds ou engines
x86_64 para a Jetson. Leia a [instalação](docs/installation.md) para preparar
a composição desejada.

## Configurar um workspace novo

Os comandos pressupõem ROS Humble e ferramentas de desenvolvimento preparados.
Use diretório novo; em workspace existente, confira as árvores antes de importar.

~~~bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws
git clone https://github.com/escuderia-poliposition-usp/ebva.git src/ebva
vcs import --input src/ebva/dependencies.repos src
git -C src/external/urg_node2 submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon list
rosdep check --from-paths src --ignore-src
~~~

Resolva as dependências da composição antes do build. rosdep check diagnostica
pendências; não instala SDKs, pesos ou mapas. Não acrescente f1tenth_system sobre
a receita: ele pode duplicar VESC/teleop_tools.

Para a composição completa, com todas as dependências preparadas:

~~~bash
colcon build --symlink-install
source install/local_setup.bash
~~~

Para uma primeira verificação menor, somente com dependências das interfaces:

~~~bash
colcon build --symlink-install --packages-up-to ebva_control_msgs lane_msgs ebva_control_contracts
source install/local_setup.bash
ros2 interface show lane_msgs/msg/LaneBoundaries
~~~

Esse build menor não instala todo o bringup. --packages-up-to inclui dependências
locais; --packages-select pressupõe dependências instaladas. Consulte
[a instalação](docs/installation.md) e
[primeiros passos na Wiki](https://github.com/escuderia-poliposition-usp/ebva/wiki/Primeiros-passos).

## Usar e escolher uma composição

Em um workspace com bringup e componentes instalados, abra um terminal novo:

~~~bash
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 pkg prefix ebva_bringup
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

O prefixo deve apontar para o workspace escolhido. check_only resolve o perfil
e recursos sem iniciar componentes, consultar DDS ou abrir dispositivos.
Não comprova disponibilidade do hardware ou conteúdo de mapas/modelos.

| Quero… | Perfil ou entrada | Próxima leitura |
| --- | --- | --- |
| Comandar com RC | manual | [Autoridade](docs/command-authority.md) |
| Teclado com RC prioritário | keyboard | [Veículo](control/ebva_vehicle_control/README.md) |
| Adquirir sensores | sensors | [Sensores](sensors/ebva_sensors/README.md) |
| Extrair faixas / detectar placas | lanes, signs, perception | [Percepção na Wiki](https://github.com/escuderia-poliposition-usp/ebva/wiki/Percepcao) |
| Mapear / localizar | slam_mapping_*, localization_* | [Localização](localization/ebva_localization/README.md) |
| Navegar por mapa | navigation_slam, navigation_amcl | [Navegação](navigation/ebva_navigation/README.md) |
| Controlar por classificação | supervisor, supervisor_* | [Supervisor](control/ebva_supervisor/README.md) |
| Gravar uma sessão | ebva_recording/recording.launch.py | [Gravação](recording/ebva_recording/README.md) |
| Verificar integrações sem dispositivos | ebva_system_tests/run_all | [Testes](docs/testing.md) |

Após conferir configuração e condições físicas, a execução manual é:

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual
~~~

**Manual abre RC/VESC reais** e pode habilitar atuação uma vez após RC válido e
neutro; autonomia permanece desligada. Execute uma composição principal por vez:
mapeamento/localização/navegação já incluem a base manual. Gravação é separada.
Veja [operação](docs/operation.md) e
[perfis na Wiki](https://github.com/escuderia-poliposition-usp/ebva/wiki/Perfis-e-configuracao).

## Componentes

Os diretórios de primeiro nível agrupam responsabilidades; cada diretório com
package.xml é uma unidade ROS. Os READMEs abaixo descrevem objetivo, estrutura,
interfaces, dependências e uso.

| Área | Pacote | Função |
| --- | --- | --- |
| bringup | [ebva_bringup](bringup/ebva_bringup/README.md) | Composição por perfis |
| calibration | [ebva_calibration](calibration/ebva_calibration/README.md) | Intrínsecos e pares sincronizados |
| control | [ebva_cbf_control](control/ebva_cbf_control/README.md) | Filtros por funções de barreira |
| control | [ebva_control_contracts](control/ebva_control_contracts/README.md) | Validade temporal compartilhada |
| control | [ebva_frm_control](control/ebva_frm_control/README.md) | Controle reativo por distâncias |
| control | [ebva_lane_keeping](control/ebva_lane_keeping/README.md) | Controle por faixas |
| control | [ebva_mapless_control](control/ebva_mapless_control/README.md) | Controle sem mapa por LiDAR |
| control | [ebva_path_control](control/ebva_path_control/README.md) | Seguimento de trajetória |
| control | [ebva_supervisor](control/ebva_supervisor/README.md) | Controle por classificação de pista |
| control | [ebva_vehicle_control](control/ebva_vehicle_control/README.md) | Aquisição RC, autoridade e atuação |
| diagnostics | [ebva_analysis](diagnostics/ebva_analysis/README.md) | Análise de bags, mapas e logs |
| diagnostics | [ebva_diagnostics](diagnostics/ebva_diagnostics/README.md) | Observação de estado |
| interfaces | [ebva_control_msgs](interfaces/ebva_control_msgs/README.md) | Mensagens de controle e autoridade |
| interfaces | [lane_msgs](interfaces/lane_msgs/README.md) | Geometria das faixas |
| localization | [ebva_localization](localization/ebva_localization/README.md) | Estado, TF e mapas |
| navigation | [ebva_navigation](navigation/ebva_navigation/README.md) | Nav2 e ponte cinemática |
| navigation | [ebva_reference_tools](navigation/ebva_reference_tools/README.md) | Referências de trajetória |
| perception | [ebva_lane_detection](perception/ebva_lane_detection/README.md) | Percepção de faixas por segmentação |
| perception | [ebva_lane_hough](perception/ebva_lane_hough/README.md) | Biblioteca de percepção Hough |
| perception | [ebva_pointcloud_fusion](perception/ebva_pointcloud_fusion/README.md) | Projeção e fusão de nuvens |
| perception | [ebva_sign_depth](perception/ebva_sign_depth/README.md) | Segmentação clássica e profundidade |
| perception | [ebva_sign_detection](perception/ebva_sign_detection/README.md) | Detecção de placas por YOLO |
| recording | [ebva_recording](recording/ebva_recording/README.md) | Gravação e monitoramento |
| sensors | [ebva_sensors](sensors/ebva_sensors/README.md) | Aquisição e unidades de sensores |
| simulation | [ebva_autodrive](simulation/ebva_autodrive/README.md) | Ponte ROS do AutoDRIVE |
| simulation | [ebva_simulation](simulation/ebva_simulation/README.md) | Fontes sintéticas e modelo cinemático |
| testing | [ebva_system_tests](testing/ebva_system_tests/README.md) | Campanhas de software e replays |

Fornecedores ficam em src/external no workspace, fora deste Git. Build/install/log,
mapas, modelos, bags e resultados também não pertencem às fontes do EBVA.

## Como as peças se conectam

~~~mermaid
flowchart LR
  Sensores --> Estado[Percepção e estado]
  Referencias[Referência] --> Controle[Controlador escolhido]
  Estado --> Controle
  Controle --> Adaptador[Adaptador específico]
  RC[Aquisição RC] --> Autoridade
  Adaptador --> Autoridade
  Autoridade --> Backend[Backend e limites]
  Backend --> VESC[Driver VESC]
~~~

O perfil escolhe as ligações. Mapless dispensa localização global; supervisor
exige classificador externo; trajetória exige referência compatível. Esforço,
velocidade, aceleração e comando de atuador têm unidades/sinais diferentes.
Leia [arquitetura](docs/architecture.md), [interfaces](docs/interfaces.md) e
os contratos do método antes de combinar componentes.

## Testar, diagnosticar e contribuir

Os [testes](docs/testing.md) incluem suítes por pacote e 16 campanhas de software,
com domínio isolado e destinos novos. Mudanças em mensagens/bibliotecas exigem
verificar consumidores. [Diagnóstico](docs/troubleshooting.md) e
[Wiki](https://github.com/escuderia-poliposition-usp/ebva/wiki/Diagnostico) ajudam
a distinguir ambiente, perfil, entrada e contrato.

Testes sintéticos não demonstram calibração, RF, watchdog, frenagem ou desempenho
no veículo. direct_commands inicia somente VESC, aceitando comandos finais sem
prioridade RC ou lease do backend; o contrato está no [pacote de veículo](control/ebva_vehicle_control/README.md).

Para desenvolver, consulte [CONTRIBUTING](CONTRIBUTING.md) e o
[padrão documental](docs/documentation.md). Capacidades futuras, incluindo Gym
e preparação Docker WSL, estão no [roadmap](docs/roadmap.md).

## Licenças

O código de autoria da equipe usa [MIT](LICENSE). Componentes com licenças
próprias e avisos preservados estão em [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES.md).
