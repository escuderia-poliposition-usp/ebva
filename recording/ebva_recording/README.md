# ebva_recording — gravação e monitoramento

Gravação por rosbag2 e ROS launch Humble/Python 3.10. A coleta é iniciada
explicitamente e não depende de iniciar ou encerrar o stack do veículo.

~~~bash
ros2 launch ebva_recording recording.launch.py
ros2 launch ebva_recording recording.launch.py preset:=/dados/preset.yaml duration_s:=120 output_root:=/dados/rosbags
~~~

O launch recebe preset (default config/images.yaml), duration_s e output_root.
Vazios conservam o preset. Standalone true é padrão: encerra o próprio launch
ao concluir. Uma composição que o inclua pode usar standalone false.

## Presets e saídas

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

## Supervisão

O preflight estático verifica preset, mensagens, recorder/plugin, permissões e
espaço sem criar DDS/arquivos. A sessão verifica descoberta/tipo único e rejeita
outro monitor de métricas no domínio. Lock por usuário/host/domínio impede duas
sessões desta entrada.

Duração, falha ou reserva de espaço atingida encerram recorder/monitor.
Falha da coleta não solicita desligamento de drivers de outra composição.
Queda de energia/SIGKILL podem impedir fechamento ordenado.

## Build e testes

~~~bash
colcon build --packages-select ebva_recording --symlink-install
source install/local_setup.bash
colcon test --base-paths src/ebva --packages-select ebva_recording --executor sequential --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
~~~

O teste de bag usa tópico/métricas sintéticos. Dois casos exigem uma instalação
isolada do pacote sobre ROS Humble, com PYTHONNOUSERSITE=1: execute a
[receita de build/test isolado](../../docs/testing.md), que prepara o ambiente
antes de habilitar CANDIDATES_INSTALLED=1. A flag sozinha não satisfaz o requisito.
Use domínio exclusivo. Mantenha bags e resultados fora do Git.
