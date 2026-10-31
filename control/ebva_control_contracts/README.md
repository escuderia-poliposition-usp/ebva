# ebva_control_contracts — validade temporal compartilhada

## Objetivo

Fornecer a biblioteca Python de validade temporal usada por controladores e adaptadores, sem criar nós, tópicos ou dependência de ROS em execução. Combina idade original da medida e prazo monotônico local.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_control_contracts/](ebva_control_contracts/) | Módulos Python, algoritmos e adaptadores da função. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [measurement.py](ebva_control_contracts/measurement.py): implementa MeasurementLease sem I/O.

## Interfaces

`ebva_control_contracts.measurement.MeasurementLease(timeout, frame, future_slack=0.02)` expõe `observe(stamp, ros_now, monotonic_now, frame)`, `fresh(ros_now, monotonic_now)` e `clear()`. Tempos são segundos; o consumidor trata o retorno booleano e a perda de validade.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** biblioteca padrão Python; sem dependência ROS em execução.
- **Build:** `ament_python`. **Testes:** `python3-pytest`.

## Uso e configuração

Use tempos numéricos da mesma época e frame do produtor:

~~~python
from ebva_control_contracts.measurement import MeasurementLease

lease = MeasurementLease(timeout=0.2, frame="camera_lane")
accepted = lease.observe(stamp=10.0, ros_now=10.05,
                         monotonic_now=50.0, frame="camera_lane")
assert accepted and lease.fresh(ros_now=10.1, monotonic_now=50.05)
lease.clear()
~~~

Stamps zero, repetidos/reordenados, não finitos, antigos, futuros além da tolerância e frames divergentes são recusados. Uma recusa de `observe` precisa ser tratada pelo consumidor. A biblioteca não publica parada, converte unidades ou decide rearme.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_control_contracts
source install/local_setup.bash
colcon test --packages-select ebva_control_contracts --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
