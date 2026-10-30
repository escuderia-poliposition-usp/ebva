# ebva_simulation — fontes e modelos de software

O pacote oferece fontes sintéticas e modelo cinemático para composições ROS,
separados dos componentes da aplicação.

| Interface | Função |
| --- | --- |
| synthetic_camera_publisher | Publica cena sintética para o pipeline de faixas |
| hough_synthetic_camera | Entrada de geração sintética para comparações Hough |
| hough_camera_fixture | Publica uma imagem artificial de faixas em /camera/image_raw |
| sim_kinematic | Modelo cinemático com parâmetros explícitos |

kinematic.launch.py fornece o modelo isolado, com params_file/default em
config/kinematic.yaml. Ele não calibra o carro.

~~~bash
ros2 launch ebva_simulation kinematic.launch.py
~~~

A seleção de uma fonte para testes é explícita. Imagens/modelos sintéticos
permitem verificar contratos de software, sem demonstrar realismo de câmera,
dinâmica ou precisão física.

A integração AutoDRIVE pertence a ebva_autodrive. A integração F1TENTH Gym é
uma capacidade futura descrita no [roadmap](../../docs/roadmap.md).
O recurso sim_navigate.rviz pode ser usado para visualização de uma composição
preparada, sem iniciar componentes por si.

~~~bash
colcon build --packages-up-to ebva_simulation --symlink-install
colcon test --packages-select ebva_simulation
colcon test-result --verbose
~~~
