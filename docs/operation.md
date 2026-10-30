# Perfis e operação

## Preparar e inspecionar

No workspace construído:

~~~bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

`bringup.launch.py` é a entrada pública; sem profile, o padrão é manual.
`check_only:=true` resolve o perfil e referências explícitas sem executar
callbacks dos componentes, consultar DDS ou abrir dispositivos.

Use nome instalado ou caminho absoluto de YAML. O formato está no
[README do bringup](../bringup/ebva_bringup/README.md).

## Escolher a composição

| Perfil/família | Composição |
| --- | --- |
| manual | Aquisição RC, autoridade, backend e VESC em /ebva |
| keyboard | Teclado SSH com cadeia manual e RC prioritário |
| direct_commands | Somente VESC para comandos diretos de ensaio |
| sensors | LiDAR e câmera |
| lanes | Câmera e faixas |
| signs | Câmera e placas |
| perception | Câmera, faixas e placas |
| slam_mapping_zed / imu / ekf | Base manual e sensores/estado/mapeamento escolhidos |
| localization_slam / amcl | Base manual, sensores e localização por mapa |
| navigation_slam / amcl | Localização, Nav2, ponte e adaptador autônomo |
| supervisor e supervisor_* | Controlador de supervisão ou método fixo |

Execute uma composição principal por vez. Navegação/mapeamento já incluem
a base manual; não inicie outra manual ao lado.

Manual sem check_only abre hardware real e pode habilitar atuação após RC
neutro. Veja [autoridade](command-authority.md).

## Configurar nos donos

- Veículo: portas, centro, limites e partida em ebva_vehicle_control.
- Teclado: eixos, lease e parâmetros key_teleop em ebva_vehicle_control.
- Sensores: drivers, tópicos e extrínsecos em ebva_sensors.
- Mapas: configurações SLAM/AMCL em ebva_localization.
- Modelo de placas: model_path em ebva_sign_detection.
- Controladores: parâmetros e recursos no pacote do método.

Navegação mantém conversão física desabilitada enquanto qualified/calibration_id
não expressarem uma calibração definida. Mapas/modelos vazios são entradas
exigidas; não substitua por recursos inventados para passar a inspeção.

## Executar e encerrar

Depois de conferir recursos, ambiente e condições físicas:

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual
~~~

A partida automática manual não habilita autonomia. Ctrl+C solicita encerramento;
a resposta de firmware/veículo exige validação própria.

Gravação é separada: use [ebva_recording](../recording/ebva_recording/README.md)
quando desejar bags. Iniciar um perfil não inicia coleta automaticamente.

## Composições de software

Campanhas e fontes sintéticas ficam em ebva_system_tests e ebva_simulation.
Os perfis supervisor iniciam controladores e exigem classificação externa,
sem incluir atuação física implicitamente. A campanha supervisor_software
compõe entradas, autoridade/backend e conversões sintéticas.
