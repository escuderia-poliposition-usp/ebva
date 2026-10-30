# ebva_analysis

Ferramentas de análise offline e loggers ROS. Não inicia sensores nem atuação.
Os coletores recebem tópicos públicos e exigem destinos explícitos; arquivos
existentes são recusados.

## Análise offline

~~~bash
ros2 run ebva_analysis bag_plotter_odom /caminho/bag --topic /odom -o /caminho/trajetoria.pdf
ros2 run ebva_analysis map_to_pdf /caminho/mapa.pcd -o /caminho/mapa.pdf --style filled
~~~

bag_plotter_odom lê nav_msgs/Odometry e exporta PDF, PNG ou SVG sem janela.
--storage-id seleciona o plugin de bag, sqlite3 por padrão.
map_to_pdf lê PCD ASCII/binário não comprimido e oferece estilos filled/scatter,
resolução de grade, densidade e tamanho dos marcadores. A projeção XY conserva
as coordenadas do arquivo; não infere origem geográfica ou extrínsecos.

## Coleta ROS

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
