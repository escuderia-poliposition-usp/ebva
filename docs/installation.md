# Instalação nativa

## Requisitos

O ambiente do carro é Ubuntu 22.04, ROS 2 Humble e Python 3.10. Prepare Git,
compilador/CMake, colcon e uma ferramenta compatível com o formato `.repos`.
O EBVA não instala ou atualiza JetPack, kernel ou SDKs do host.

As dependências ROS/Python ficam nos manifestos de cada pacote. A composição
completa utiliza VESC, teleop_tools, urg_node2, Nav2, slam_toolbox,
robot_localization e bibliotecas de imagem/nuvens. O wrapper ZED requer SDK
compatível com a plataforma. AutoDRIVE possui bibliotecas Python e simulador
externo próprios; consulte seu README.

Pesos de detecção, mapas e dados de calibração são entradas explícitas.
Não copie builds ou ambientes virtuais entre x86_64 e ARM64.

## Workspace novo

Prepare um diretório novo. Não importe fornecedores sobre checkouts existentes
com alterações.

~~~bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws
git clone https://github.com/escuderia-poliposition-usp/ebva.git src/ebva
vcs import --input src/ebva/dependencies.repos src
git -C src/external/urg_node2 submodule update --init --recursive
~~~

A receita obtém repositórios independentes sob `src/external`. Não é necessário
clonar f1tenth_system por cima deles. Evite descobrir duas fontes com o mesmo
nome de pacote ROS.

Instale os requisitos da composição escolhida. Em host com rosdep preparado,
`rosdep check --from-paths src --ignore-src` ajuda a identificar dependências
pendentes. SDK ZED, backend neural e simuladores seguem os respectivos provedores
e os pacotes consumidores.

## Construir e carregar

Depois de preparar as dependências:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
~~~

Para iterar em uma função:

~~~bash
colcon build --symlink-install --packages-up-to ebva_vehicle_control ebva_bringup
source install/local_setup.bash
~~~

`--packages-up-to` inclui dependências encontradas no workspace.
`--packages-select` seleciona somente os nomes informados; dependências precisam
estar instaladas. Reconstrua também os consumidores afetados por mudanças em
mensagens ou bibliotecas. Entry points, recursos, C++ e IDL exigem build.

## Conferir

~~~bash
ros2 pkg prefix ebva_bringup
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

A inspeção resolve perfil e referências explícitas sem iniciar RC, VESC,
câmera, LiDAR ou navegação. O ambiente Docker de desenvolvimento no WSL é
mantido separadamente; esta instalação não depende dele.
