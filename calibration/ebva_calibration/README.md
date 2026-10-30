# ebva_calibration — intrínsecos e coleta sincronizada

- camera_intrinsics estima intrínsecos por imagens de um tabuleiro conhecido.
- synchronized_pairs salva imagens e scans associados por timestamp.

Ambos exigem configuration_file apontando para um YAML do usuário.
Caminhos relativos resolvem em relação a esse arquivo.

~~~bash
ros2 run ebva_calibration camera_intrinsics --ros-args -p configuration_file:=/dados/calibracao.yaml
ros2 run ebva_calibration synchronized_pairs --ros-args -p configuration_file:=/dados/calibracao.yaml
~~~

## Configuração

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

## Build e testes

~~~bash
colcon build --packages-up-to ebva_calibration --symlink-install
colcon test --packages-select ebva_calibration
colcon test-result --verbose
~~~

Testes sintéticos não substituem imagens reais. Dados e resultados ficam fora
dos fontes.
