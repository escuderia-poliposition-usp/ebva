# ebva_cbf_control

Três modelos CBF preservados: yaw_rate, yaw_rate_margin e steering_tangent.
Kernels, solver, ganhos e oráculos anteriores foram preservados nesta retirada.

## Interfaces de execução

Produtores com contratos estampados:
- yaw_rate_stamped
- yaw_rate_margin_stamped
- steering_tangent_stamped

Filtros da interface anterior, mantidos como alternativas de algoritmo:
- yaw_rate_filter
- yaw_rate_margin_filter
- steering_tangent_filter

A interface select.launch.py seleciona somente um filtro:
~~~bash
ros2 launch ebva_cbf_control select.launch.py model:=yaw_rate
~~~

rc_input e vehicle_output foram retirados. Fornecer esses argumentos produz erro
explícito; a interface não volta a abrir serial ou publicar VESC diretamente.
Os três *_nominal e os três *_vehicle não são mais instalados.

## Rota de controle

Leitura RC, arbitragem e atuação pertencem a ebva_vehicle_control.
Os perfis de ensaio atuais usam produtor *_stamped, adaptador específico,
command_authority e vesc_backend. Um perfil físico depende de fontes, unidades,
geometria e calibração qualificadas; não foi criado ou habilitado nesta retirada.
Os filtros legados ainda publicam os dois Float32 AutoDRIVE. Esse dialeto
não deve ser tratado como par de comandos físicos universais.

## Limpeza das fontes aposentadas

Em 30/10/2026, os seis arquivos dos leitores/pontes aposentados foram removidos
definitivamente, incluindo as cópias de documentação, build e instalação.
Não há diretório docs/retired nem carregador de fontes históricas neste pacote.
Os backups externos estão sob responsabilidade do usuário.

Os testes ativos de projeção dos filtros e os testes numéricos/AST dos kernels
continuam usando os oráculos existentes. Nenhum algoritmo ativo foi removido.
Os testes que dependiam das pontes apagadas e a dependência pyserial sem uso
também foram retirados. O pacote instala apenas os seis executáveis listados acima.

## Contratos e limites

- [Contratos de software e caracterização](docs/cbf-software-contract.md).
- [Contrato yaw_rate estampado](docs/yaw-rate-ros-contract.md).
- [Contratos margin e steering_tangent](docs/alternate-model-ros-contracts.md).
- [Atribuições e licenças](../../THIRD_PARTY_NOTICES.md).

O solver Hildreth preservado não fornece certificação física de segurança.
As variantes e a conversão tan(ângulo) do modelo tangent continuam distintas.
Entradas inválidas, antigas, inconsistentes ou QP inadmissível são rejeitadas
pelas portas estampadas. A retirada não altera esse comportamento.
