# ebva_sensors

Interfaces ROS de aquisição LiDAR/ZED e normalização de unidades IMU.
Os métodos de filtragem e conversão são separados dos estimadores e controladores.

Launches: lidar, lidar_mapping_zed, lidar_mapping_imu, zed, zed_odometry,
zed_measurements e imu_si. A configuração pertence a config/ e é selecionada por
params_file nos launches correspondentes. O perfil de bringup escolhe quais
interfaces executar.

zed_odometry publica odometria e seu TF; zed_measurements disponibiliza medidas
para uma composição que atribua esse TF ao EKF. Uma composição deve ter um
responsável por cada TF. O filtro LiDAR conserva seus métodos numéricos e permite
input_topic/output_topic mantendo os defaults.

Executáveis: escuderia_imu, laser_filer_escuderia e vesc_imu_to_si. A conversão
vesc_imu_to_si não é um estimador de pose; sua saída deve ser associada a unidades,
eixos e extrínsecos adequados ao perfil. Não confundir valores do modelo com
calibração da montagem.

Fornecedores: urg_node2 para LiDAR e zed_wrapper para ZED. SDK ZED, GPU e
compatibilidade com o sistema do alvo são requisitos próprios da composição.
Instalar ou compilar o pacote não inicia aquisição; executar seus launches pode
abrir dispositivos. Testes de software usam medidas sintéticas.
