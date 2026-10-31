# ebva_recording — gravação e monitoramento

## Objetivo

Gravar tópicos ROS e métricas de recursos numa sessão explícita, independente do stack. Presets definem dados, duração, descoberta, segmentação e reserva de espaço.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_recording/](ebva_recording/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [configuration.py](ebva_recording/configuration.py): schema/presets e validações.
- [session.py](ebva_recording/session.py): ciclo de gravação e supervisão.
- [resource_publisher.py](ebva_recording/resource_publisher.py): publicação das métricas.

## Interfaces

`recording.launch.py`: `preset`, `duration_s`, `output_root`, `standalone`. Presets v2 em config. `resource_monitor` publica `/system/resource_usage` (`statistics_msgs/MetricsMessage`) e `/system/resource_snapshot`, `/system/resource_metadata` (`String`). A bag é saída local fora do Git.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `ament_index_python`, `launch`, `launch_ros`, `ros2launch`, `rclpy`, `statistics_msgs`, `std_msgs`, `builtin_interfaces`, `ros2bag`, `rosbag2_transport`, `rosbag2_py`, `rosbag2_storage_default_plugins`, `rosidl_runtime_py`, `python3-yaml`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

rosbag2 exige o plugin de storage selecionado. Tegrastats é opcional no host e sua ausência é informada. A sessão não inicia produtores dos tópicos gravados.

## Uso e configuração

~~~bash
ros2 launch ebva_recording recording.launch.py
ros2 launch ebva_recording recording.launch.py preset:=/dados/preset.yaml duration_s:=120 output_root:=/dados/rosbags
~~~

O launch recebe preset (default config/images.yaml), duration_s e output_root.
Vazios conservam o preset. Standalone true é padrão: encerra o próprio launch
ao concluir. Uma composição que o inclua pode usar standalone false.

### Presets e saídas

Presets version 2 declaram tópicos absolutos únicos, duração, segmentação,
cache, métricas, descoberta, reserva de espaço e destino. Defaults: 60 s,
segmentos 1 GiB, cache 100 MiB, métricas 1 s, reserva 2048 MiB e ~/rosbags.
Duração zero segue até encerramento.

| Preset | Conteúdo |
| --- | --- |
| images | Imagem retificada, faixas e debug |
| reference | Scan filtrado e odometria EKF |
| sensors | Scan filtrado e imagem |
| signs | Imagem, detecções e imagem anotada de placas |

/system/resource_usage, /system/resource_snapshot e /system/resource_metadata
são registrados na mesma bag. Metadados incluem preset efetivo, sessão e
recursos; não há CSV paralelo. Tegrastats é opcional e sua ausência é informada.

Confira tópicos/tipos dos produtores da composição antes de escolher o preset.

### Supervisão

O preflight estático verifica preset, mensagens, recorder/plugin, permissões e
espaço sem criar DDS/arquivos. A sessão verifica descoberta/tipo único e rejeita
outro monitor de métricas no domínio. Lock por usuário/host/domínio impede duas
sessões desta entrada.

Duração, falha ou reserva de espaço atingida encerram recorder/monitor.
Falha da coleta não solicita desligamento de drivers de outra composição.
Queda de energia/SIGKILL podem impedir fechamento ordenado.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_recording
source install/local_setup.bash
colcon test --packages-select ebva_recording --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

O teste de bag usa tópico/métricas sintéticos. Dois casos exigem uma instalação
isolada do pacote sobre ROS Humble, com PYTHONNOUSERSITE=1: execute a
[receita de build/test isolado](../../docs/testing.md), que prepara o ambiente
antes de habilitar CANDIDATES_INSTALLED=1. A flag sozinha não satisfaz o requisito.
Use domínio exclusivo. Mantenha bags e resultados fora do Git.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
