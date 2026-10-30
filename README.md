# EBVA — Escuderia Brasileira de Veículos Autônomos

O EBVA reúne os componentes ROS 2 do carro F1TENTH da Escuderia Poliposition USP:
sensores, percepção, localização, controle, atuação, gravação e análise.
Controladores e métodos alternativos permanecem identificados pelos seus
contratos. Perfis YAML escolhem os componentes e referenciam configurações
mantidas pelos respectivos pacotes.

O carro utiliza **ROS Humble nativo em Ubuntu 22.04/Python 3.10**. O ambiente
Docker de desenvolvimento no WSL é preparado separadamente.

## Começar

Em um workspace construído:

~~~bash
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

`check_only` inspeciona o perfil e seus recursos sem iniciar componentes.
O perfil padrão `manual`, executado sem essa opção, abre RC e VESC reais.
Seu YAML habilita atuação manual uma vez após RC válido e neutro; a autonomia
permanece desligada. Leia a [operação](docs/operation.md) antes de executar.

- [Instalação nativa e dependências](docs/installation.md)
- [Arquitetura](docs/architecture.md) e [interfaces](docs/interfaces.md)
- [Perfis e operação](docs/operation.md)
- [Testes de software](docs/testing.md) e [diagnóstico](docs/troubleshooting.md)
- [Wiki: tutoriais e exemplos](https://github.com/escuderia-poliposition-usp/ebva/wiki)
- [Contribuição](CONTRIBUTING.md) e [roadmap](docs/roadmap.md)

## Organização

| Diretório | Responsabilidade |
| --- | --- |
| bringup | Orquestração por perfis ROS launch/YAML |
| interfaces | Mensagens compartilhadas com semântica explícita |
| control | Controladores, conversões, aquisição RC, autoridade e backend |
| sensors | Interfaces e unidades dos sensores |
| perception | Faixas, placas, profundidade e fusão de nuvens |
| localization | Estimadores, estado/TF e ferramentas de mapas |
| navigation | Navegação e ferramentas de referência |
| recording | Gravação e monitoramento explícitos |
| diagnostics | Diagnóstico e análise |
| calibration | Calibração e coleta sincronizada |
| simulation | Integrações de simulador e fontes sintéticas |
| testing | Campanhas, replays e verificações de integração |

Cada pacote possui suas fontes, dependências, recursos e testes. Build e
execução usam `colcon` e interfaces ROS; não dependem de checkout da bancada
ou de ferramentas de outro projeto. Fornecedores ficam em `src/external`,
obtidos pela receita [dependencies.repos](dependencies.repos).

Instalar um pacote não inicia seus nós. O build seleciona pacotes; o perfil
seleciona a composição executada. SDK, mapas, modelos e simuladores acompanham
os requisitos da função escolhida.

## Limites de uso

Testes sintéticos e replays verificam software. Calibração, RF, watchdog de
firmware, frenagem, geometria e comportamento no veículo exigem ensaios próprios.
Mapas e modelos externos são entradas explícitas; os arquivos volumosos ficam
fora do repositório.

`direct_commands` inicia somente o driver VESC e permite comandos diretos.
Ele não oferece prioridade RC ou lease do backend; consulte a
[interface de veículo](control/ebva_vehicle_control/README.md).

## Licenças

O código de autoria da equipe usa [MIT](LICENSE). Componentes com licenças
próprias e seus avisos estão em [atribuições](THIRD_PARTY_NOTICES.md).
