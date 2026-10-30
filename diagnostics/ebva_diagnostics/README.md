# ebva_diagnostics — observação de estado

Ferramentas ROS que não publicam comandos de atuação ou novas estimativas.

## Interfaces

`map_odon_distance` consulta TF map→odom, publica /map_odom_distance e
acrescenta linhas a um arquivo. Defaults: log_directory /tmp e log_filename
map_odom_distance.txt. A escrita faz parte de sua execução.

`odometry_viewer` assina /odometry/pose_source/filtered (nav_msgs/Odometry)
e apresenta valores no terminal com rich.

~~~bash
ros2 run ebva_diagnostics odometry_viewer
~~~

Selecione explicitamente as ferramentas e confira os frames/timestamps
da composição observada.

## Build e testes

~~~bash
colcon build --packages-up-to ebva_diagnostics --symlink-install
colcon test --packages-select ebva_diagnostics
colcon test-result --verbose
~~~

Mantenha diagnósticos fora do Git. Import/build não executam a coleta.
