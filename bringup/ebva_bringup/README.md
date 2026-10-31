# ebva_bringup — composição por perfis

## Objetivo

Compor funções do carro por perfis YAML e localizar configurações instaladas nos respectivos pacotes. É a entrada de operação do monorepo; algoritmos e defaults continuam nos donos.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_bringup/](ebva_bringup/) | Módulos Python, algoritmos e adaptadores da função. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [profiles/](profiles/) | Composições YAML instaladas. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [profiles.py](ebva_bringup/profiles.py): valida o esquema e resolve recursos.
- [launching.py](ebva_bringup/launching.py): apoia o ciclo de execução da composição.

## Interfaces

`bringup.launch.py`: `profile` aceita nome instalado ou YAML absoluto (padrão `manual`); `check_only:=true` resolve a composição sem iniciar componentes. Perfis v1 declaram `launches`, `nodes` e referências `{package, file}`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** [ebva_vehicle_control](../../control/ebva_vehicle_control/README.md), [ebva_sensors](../../sensors/ebva_sensors/README.md), [ebva_localization](../../localization/ebva_localization/README.md), [ebva_navigation](../../navigation/ebva_navigation/README.md), [ebva_lane_detection](../../perception/ebva_lane_detection/README.md), [ebva_sign_detection](../../perception/ebva_sign_detection/README.md).
- **ROS e bibliotecas externas:** `ament_index_python`, `launch`, `launch_ros`, `ros2launch`, `rclpy`, `python3-yaml`.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

O manifesto declara as famílias básicas; referências opcionais de perfil, como ebva_supervisor, também exigem o pacote instalado. Resolver um perfil não instala seus componentes.

## Uso e configuração

### Inspeção

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

Sem profile, o padrão é manual. Check_only resolve YAML e recursos explícitos
sem executar callbacks, consultar DDS ou abrir dispositivos. Defaults omitidos
aparecem como default do pacote.

Manual sem check_only abre RC e VESC reais. Seu YAML habilita atuação manual
uma vez após RC válido/neutro, mantendo autonomia off.
Veja [operação](../../docs/operation.md).

### Perfis

Manual, keyboard e direct_commands usam /ebva. Sensores, percepção, SLAM,
localização e navegação conservam os namespaces declarados. Navegação e
mapeamento já incluem a base manual; execute uma composição principal por vez.

Lanes compõe câmera/faixas; signs câmera/placas; perception as três.
Supervisor/supervisor_* iniciam somente o controlador escolhido e exigem
classificação externa, sem atuação física implícita.

Use nome instalado ou caminho absoluto de YAML. Mapas, modelos e ganhos
são escolhidos nas configurações dos donos.

### Esquema YAML v1

- version 1, name e namespace opcional.
- launches: id/package/file e arguments opcionais.
- nodes: id/package/executable, name opcional e parameters opcionais.
- Cada argumento de launch e elemento de parameters é uma referência
  `{package, file}` relativa ao share instalado.

Não há herança, parâmetros inline, ros__parameters ou paths livres de
mapas/modelos no perfil. Acrescente/remova componentes e referencie
a configuração de seu dono.

~~~yaml
version: 1
name: lidar_example
namespace: /
launches:
  - id: lidar
    package: ebva_sensors
    file: launch/lidar_mapping_zed.launch.py
    arguments:
      params_file:
        package: ebva_sensors
        file: config/lidar_mapping_zed.yaml
~~~

Após salvar o YAML, inspecione seu caminho absoluto com check_only antes
de executar componentes.

### Teclado, ensaio e gravação

Keyboard exige terminal SSH interativo, RC prioritário válido e RC/teclado
neutros na partida. A configuração key_teleop pertence a ebva_vehicle_control.

Direct_commands inicia só VESC, sem RC, autoridade/backend ou lease de atuação.
A [interface de veículo](../../control/ebva_vehicle_control/README.md)
documenta os dois modos.

Gravação pertence a ebva_recording. Campanhas pertencem a ebva_system_tests
e utilizam este mesmo esquema.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_bringup
source install/local_setup.bash
colcon test --packages-select ebva_bringup --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Os componentes referenciados pelo perfil também precisam estar instalados.
Construir o orquestrador não instala SDKs, mapas, modelos ou os pacotes
de cada composição.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
