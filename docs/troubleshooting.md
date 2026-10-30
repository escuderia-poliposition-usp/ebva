# Diagnóstico

## Pacote ou recurso não encontrado

~~~bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 pkg prefix ebva_bringup
~~~

Overlays antigos podem selecionar outra instalação. Não carregue duas fontes
com o mesmo nome ROS. Recursos, entry points, mensagens e C++ modificados
exigem build; reconstrua também consumidores afetados.

## Perfil rejeitado

Use check_only e confira package/file, executável e recurso instalado.
Argumentos e parâmetros no perfil referenciam arquivos dos donos.
Mapas/modelos não são parâmetros livres do perfil.

## Manual aguardando habilitação

Observe os diagnósticos. A partida automática exige grafo consistente e RC
válido/recente/neutro em MANUAL. Motivos incluem startup_waiting_for_graph e
startup_waiting_for_neutral_rc. Um pedido explícito ao serviço cancela a
tentativa pendente. Desarme posterior não é revertido automaticamente.
Consulte [autoridade](command-authority.md) e o YAML do veículo.

## Teclado sem terminal

Keyboard exige terminal controlador; use SSH interativo ou ssh -t.
Não depende de xterm/desktop. Confira RC válido/neutro, backend habilitado,
escala e lease.

## Mapa, modelo ou dependência específica ausente

Informe mapas SLAM/AMCL e checkpoint real no detector. ZED requer SDK compatível.
AutoDRIVE requer simulador e bibliotecas de integração. Consulte o README da
função; os fontes não preparam por si todos os recursos externos.

## Replay sem mensagens

Confira domínio, localhost, QoS e /clock. Players com serviços fixos devem
rodar serialmente. Stamps antigos, futuros ou repetidos podem ser rejeitados.

## Gravação encerrada

Confira tópicos/tipos, timeout, permissões, espaço e outro monitor de métricas.
Falta de produtor ou espaço pode encerrar a coleta sem desligar os drivers.

## Ensaio físico

Identifique operador, alimentação, dispositivos e firmware. Backend e driver
têm papéis diferentes. Zero longitudinal e eco de servo não demonstram
frenagem ou ângulo real da roda.
