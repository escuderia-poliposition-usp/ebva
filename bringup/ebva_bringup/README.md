# ebva_bringup — composição por perfis

A entrada pública `bringup.launch.py` carrega perfis YAML independentes.
Eles listam launches/nós e referências a configurações instaladas nos donos.
Defaults, validações de domínio e algoritmos pertencem aos respectivos pacotes.

## Inspeção

~~~bash
ros2 launch ebva_bringup bringup.launch.py profile:=manual check_only:=true
~~~

Sem profile, o padrão é manual. Check_only resolve YAML e recursos explícitos
sem executar callbacks, consultar DDS ou abrir dispositivos. Defaults omitidos
aparecem como default do pacote.

Manual sem check_only abre RC e VESC reais. Seu YAML habilita atuação manual
uma vez após RC válido/neutro, mantendo autonomia off.
Veja [operação](../../docs/operation.md).

## Perfis

Manual, keyboard e direct_commands usam /ebva. Sensores, percepção, SLAM,
localização e navegação conservam os namespaces declarados. Navegação e
mapeamento já incluem a base manual; execute uma composição principal por vez.

Lanes compõe câmera/faixas; signs câmera/placas; perception as três.
Supervisor/supervisor_* iniciam somente o controlador escolhido e exigem
classificação externa, sem atuação física implícita.

Use nome instalado ou caminho absoluto de YAML. Mapas, modelos e ganhos
são escolhidos nas configurações dos donos.

## Esquema YAML v1

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

## Teclado, ensaio e gravação

Keyboard exige terminal SSH interativo, RC prioritário válido e RC/teclado
neutros na partida. A configuração key_teleop pertence a ebva_vehicle_control.

Direct_commands inicia só VESC, sem RC, autoridade/backend ou lease de atuação.
A [interface de veículo](../../control/ebva_vehicle_control/README.md)
documenta os dois modos.

Gravação pertence a ebva_recording. Campanhas pertencem a ebva_system_tests
e utilizam este mesmo esquema.

## Build

~~~bash
colcon build --packages-select ebva_bringup --symlink-install
~~~

Os componentes referenciados pelo perfil também precisam estar instalados.
Construir o orquestrador não instala SDKs, mapas, modelos ou os pacotes
de cada composição.
