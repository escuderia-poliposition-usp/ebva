# EBVA system tests
Campanhas determinísticas, verificações de integração, replays e probes separados
dos pacotes de execução. Os perfis YAML de ensaio selecionam apenas nós de software
e utilizam a entrada pública de ebva_bringup. Parâmetros dos nós ficam em
config/testing no dono; casos, dados sintéticos e verificações ficam aqui.
CSV de referência usado pelo controlador é recurso de ebva_path_control.

~~~bash
source /opt/ros/humble/setup.bash
source /home/poliposition/f1tenth_ws/install/local_setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184
ros2 run ebva_system_tests run_all --output /tmp/ebva-campaigns-new
~~~
O destino deve ser novo. --campaign manual seleciona uma campanha. results.json
registra cada comando/retorno. Não executar dois players com serviços fixos no
mesmo domínio. run_all nunca executa bench_probe, Hough com câmera, sensores,
drivers ou stacks SLAM/Nav2. Probes físicos permanecem explícitos e dependem de
escopo próprio. Os replays não permanecem instalados nos pacotes antigos.

Testes unitários dos algoritmos permanecem nos donos. A dependência de testes
para aplicação nunca é invertida. Ferramentas Hough são opcionais e preservam
dependências/pendências de origem, sem qualificação nesta entrega.
