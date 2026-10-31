# ebva_analysis — análise de bags, mapas e logs

## Objetivo

Converter bags/mapas em figuras e coletar tabelas de tópicos públicos. CLIs offline leem arquivos; loggers observam uma composição existente e exigem destino explícito.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_analysis/](ebva_analysis/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [bag_plotter_odom.py](ebva_analysis/bag_plotter_odom.py): figuras a partir de bags.
- [map_to_pdf.py](ebva_analysis/map_to_pdf.py): figuras de PCD.
- [logger_scan.py](ebva_analysis/logger_scan.py): coleta CSV de scan.

## Interfaces

`bag_plotter_odom`: bag `Odometry` → PDF/PNG/SVG; `map_to_pdf`: PCD → figura. `data_logger_frm`: IPS `Point` e velocidade `Float32` AutoDRIVE → CSV. `logger_cba`/`logger_scan`: `Odometry`/`LaserScan` → CSV.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `rosbag2_py`, `python3-numpy`, `python3-matplotlib`, `rosidl_runtime_py`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

### Análise offline

~~~bash
ros2 run ebva_analysis bag_plotter_odom /caminho/bag --topic /odom -o /caminho/trajetoria.pdf
ros2 run ebva_analysis map_to_pdf /caminho/mapa.pcd -o /caminho/mapa.pdf --style filled
~~~

bag_plotter_odom lê nav_msgs/Odometry e exporta PDF, PNG ou SVG sem janela.
--storage-id seleciona o plugin de bag, sqlite3 por padrão.
map_to_pdf lê PCD ASCII/binário não comprimido e oferece estilos filled/scatter,
resolução de grade, densidade e tamanho dos marcadores. A projeção XY conserva
as coordenadas do arquivo; não infere origem geográfica ou extrínsecos.

### Coleta ROS

~~~bash
ros2 run ebva_analysis data_logger_frm --ros-args -p output_file:=/caminho/ips.csv
ros2 run ebva_analysis logger_cba --ros-args -p output_file:=/caminho/odom.csv -p topic:=/odom
ros2 run ebva_analysis logger_scan --ros-args -p output_dir:=/caminho/saida -p filename:=scan.csv -p topic:=/scan
~~~

Crie o diretório de saída antes de executar. data_logger_frm recebe os tópicos
AutoDRIVE de IPS/velocidade; seu tempo é o de recepção e a velocidade é a última
amostra recebida, sem sincronização presumida. logger_cba conserva o header da
odometria no CSV. logger_scan conserva o tempo e metadados do scan, com campos
vazios para ranges/intensidades não finitos. Remapeamentos ROS podem selecionar
outras instâncias compatíveis.

Requer NumPy, Matplotlib e, para bags, rosbag2_py e rosidl_runtime_py.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_analysis
source install/local_setup.bash
colcon test --packages-select ebva_analysis --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
