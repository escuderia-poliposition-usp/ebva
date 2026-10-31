# ebva_mapless_control — controle sem mapa por LiDAR

## Objetivo

Calcular direção reativa por Follow the Gap e uma variante PD com feedback, sem exigir localização global. Cada método mantém seus parâmetros e domínio de saída.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_mapless_control/](ebva_mapless_control/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [docs/](docs/) | Contratos detalhados e exemplos. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [ftg_pd.py](ebva_mapless_control/ftg_pd.py): scan/feedback e contrato PD.
- [scan_direct.py](ebva_mapless_control/scan_direct.py): direção direta.
- [scan_pd.py](ebva_mapless_control/scan_pd.py): variante de parâmetros com direção direta.

## Interfaces

`ftg_pd`: `sensors/scan` (`LaserScan`) + `sensors/steering` (`JointState`) → `control/autonomy/ftg_pd` (`FtgPdCommand`) e `control/ftg_status` (`DiagnosticArray`). `scan_direct`/`scan_pd` usam `/scan` e feedback AutoDRIVE, publicando dois `Float32` do simulador.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_control_contracts](../ebva_control_contracts/README.md), [ebva_control_msgs](../../interfaces/ebva_control_msgs/README.md).
- **ROS e bibliotecas externas:** `python3-numpy`, `rclpy`, `diagnostic_msgs`, `sensor_msgs`, `std_msgs`, `launch`, `launch_ros`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

Selecione o método antes de compor entradas e consumidores. `ftg_pd` exige parâmetros explícitos de scan/frame/feedback; consulte [o contrato](docs/ftg-pd-contract.md) e as fixtures em config/testing.

~~~bash
ros2 run ebva_mapless_control ftg_pd --ros-args --params-file /dados/ftg.yaml
~~~

Os métodos diretos aceitam `select.launch.py method:=scan_direct` ou `method:=scan_pd`. Scan_direct mantém TTC 10/throttle máximo 0.24; FTG/PD, TTC 30/throttle 0.12; scan_pd, TTC 50/throttle 0.12. Em scan_pd, o PD da fonte está comentado: a direção permanece direta.

FtgPdCommand preserva os dois headers. Esforço PD não é ângulo final ou domínio normalizado presumido. O feedback da campanha `ftg_pd` é sintético; feedback físico/AutoDRIVE exige contrato próprio. Os métodos não iniciam aquisição RC ou driver.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_mapless_control
source install/local_setup.bash
colcon test --packages-select ebva_mapless_control --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
