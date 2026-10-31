# lane_msgs — geometria das faixas

## Objetivo

Compartilhar a geometria de faixas entre percepção e controle. O produtor atual é ebva_lane_detection; o consumidor de controle é ebva_lane_keeping.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [msg/](msg/) | Definições de mensagens ROS. |
| [package.xml](package.xml), [CMakeLists.txt](CMakeLists.txt) | Dependências e instalação de recursos/entry points. |

## Interfaces

[`LaneBoundaries`](msg/LaneBoundaries.msg) contém `std_msgs/Header header` e vetores `geometry_msgs/Point[] left_lane/right_lane`. Pontos ordenados do mais próximo ao mais distante; vetor vazio indica ausência da faixa no quadro. Não carrega confiança nem booleano de validade.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `std_msgs`, `geometry_msgs`, `rosidl_default_runtime`.
- **Build:** `ament_cmake`, `rosidl_default_generators`.

## Uso e configuração

No produtor atual, x aponta para frente, y para a direita, z é zero e a origem está na câmera. A escala métrica depende da homografia configurada. `header.stamp` conserva o instante da imagem e `header.frame_id` identifica o plano declarado.

Esse modelo não coincide automaticamente com o corpo ROS FLU. O consumidor deve conferir sinal, escala, montagem e idade; renomear o frame não transforma os pontos.

Depois de construir e carregar o overlay:

~~~bash
ros2 interface show lane_msgs/msg/LaneBoundaries
~~~

Consulte [o produtor de faixas](../../perception/ebva_lane_detection/README.md) e [o controlador](../../control/ebva_lane_keeping/README.md). Nenhum nó/launch é instalado por este pacote.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lane_msgs
source install/local_setup.bash
ros2 interface show lane_msgs/msg/LaneBoundaries
~~~

Confira geração de interfaces e reconstrua consumidores ao alterar campos/tipos. Não há suíte própria; a semântica é exercitada nos produtores/consumidores.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
