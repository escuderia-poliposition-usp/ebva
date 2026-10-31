# ebva_sensors — aquisição e unidades de sensores

## Objetivo

Encapsular launches LiDAR/ZED e fornecer filtragem de scan e adaptação de IMU. Entrega medidas para percepção/localização; estimação de pose e controle têm outros donos.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_sensors/](ebva_sensors/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [vesc_imu_to_si.py](ebva_sensors/vesc_imu_to_si.py): conversão para unidades SI.
- [laser_filer_escuderia.py](ebva_sensors/laser_filer_escuderia.py): filtro de scan.
- [config](config): variantes LiDAR/ZED/IMU.

## Interfaces

Launches `lidar*` usam urg_node2; `zed*` usam zed_wrapper. `vesc_imu_to_si` converte `Imu` de g/graus por segundo para m/s² e rad/s. `escuderia_imu` mantém processamento alternativo; `laser_filer_escuderia` filtra `LaserScan` (o nome instalado mantém `filer`). Tópicos/frames constam dos YAMLs em config.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `sensor_msgs`, `std_msgs`, `python3-numpy`, `launch`, `launch_ros`, `ament_index_python`, `python3-yaml`, `urg_node2`, `tf2_ros`, `lifecycle_msgs`, `zed_wrapper`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

urg_node2 e zed_wrapper são fornecedores de [dependencies.repos](../../dependencies.repos). ZED exige SDK/GPU compatíveis com o host. A composição define dispositivos, permissões, rede e extrínsecos.

## Uso e configuração

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

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_sensors
source install/local_setup.bash
colcon test --packages-select ebva_sensors --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
