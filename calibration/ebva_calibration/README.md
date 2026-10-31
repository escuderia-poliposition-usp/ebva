# ebva_calibration — intrínsecos e pares sincronizados

## Objetivo

Estimar intrínsecos de câmera por tabuleiro e coletar pares imagem/scan associados por timestamp. Não há calibrador extrínseco automático.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_calibration/](ebva_calibration/) | Módulos Python, algoritmos e adaptadores da função. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [intrinsics.py](ebva_calibration/intrinsics.py): calibração intrínseca.
- [synchronizer.py](ebva_calibration/synchronizer.py): captura de pares sincronizados.

## Interfaces

`camera_intrinsics`: `Image` e geometria de tabuleiro → calibração salva; `synchronized_pairs`: `Image` + `LaserScan` → PNG/NPZ por sincronização aproximada. Ambos recebem `configuration_file`; caminhos relativos resolvem a partir desse YAML.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `cv_bridge`, `message_filters`, `python3-numpy`, `python3-opencv`, `python3-yaml`, `rclpy`, `sensor_msgs`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

A captura exige câmera/scan publicados. Medidas reais do tabuleiro e dados de calibração são entradas do usuário.

## Uso e configuração

Ambos exigem configuration_file apontando para um YAML do usuário.
Caminhos relativos resolvem em relação a esse arquivo.

~~~bash
ros2 run ebva_calibration camera_intrinsics --ros-args -p configuration_file:=/dados/calibracao.yaml
ros2 run ebva_calibration synchronized_pairs --ros-args -p configuration_file:=/dados/calibracao.yaml
~~~

### Configuração

Use [a estrutura de exemplo](docs/configuration-example.yaml) e preencha as
medidas reais indicadas como null antes de executar. Os nomes de saída e tópicos
também devem corresponder ao seu ensaio.

O YAML contém camera.camera_topic, camera.image_size.width/length,
camera.chessboard.rows/columns/square_size_m, lidar.lidar_topic e general.
Rows/columns são cantos internos; square_size_m é a distância física entre eles.

Intrínsecos usa general.config_folder e general.camera_intrinsic_calibration
(nome de arquivo). Pares usa general.data_folder, slop (segundos) e
max_file_saved.

A coleta cria capture_UUID com PNGs/NPZs, ranges/angles em m/rad, stamps e frames.
Ao encerrar camera_intrinsics por Ctrl+C, o nó tenta salvar a calibração.
A calibração exige dez vistas e recusa sobrescrever a saída. Show_debug é false
por padrão. Avalie reprojeção, cobertura de vistas, dimensões e câmera reais.
Não há executável de calibração extrínseca neste pacote.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_calibration
source install/local_setup.bash
colcon test --packages-select ebva_calibration --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Testes sintéticos não substituem imagens reais. Dados e resultados ficam fora
dos fontes.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
