# Autoridade de comandos

## Composição com RC

Aquisição RC, autoridade e backend pertencem a ebva_vehicle_control.
O controlador e seu adaptador fornecem uma entrada antes da autoridade.
Somente o backend publica o par final motor/direção dessa composição.

- RC válido tem prioridade sobre o par completo.
- Com autonomia off, MANUAL permanece selecionado inclusive no neutro,
  aplicando zero longitudinal e centro configurado.
- Com autonomia on, retornar RC ao neutro retoma a entrada autônoma somente
  se RC e comando autônomo continuarem válidos.
- RC inválido/expirado bloqueia autonomia. Autonomia expirada não impede
  RC válido de assumir manualmente.
- Falta de comando selecionado válido produz STOP conforme a política;
  comandos vencidos não são conservados.

A aquisição serial atual recebe dois canais de pulso. RcInput registra
recepção e validade do protocolo; RF_UNKNOWN informa que esses dados não
distinguem perda de RF de um receptor repetindo pulsos neutros.

## Partida e desarme

O YAML manual usa `auto_enable_on_startup: true`. O backend começa sem
habilitação e aguarda grafo consistente e RC válido, recente e neutro em
MANUAL. A habilitação ocorre uma vez; autonomia permanece off.

Um pedido explícito ao serviço de atuação cancela a tentativa pendente.
Desarme posterior não é revertido automaticamente. Reiniciar a autoridade
não rearma um backend desabilitado. O executável genérico mantém default false;
a escolha da partida pertence à configuração do veículo.

O teclado usa a mesma autoridade e configuração manual. Sua seleção do canal
externo ocorre após RC/teclado neutros e backend habilitado. AUTONOMY nessa
mensagem significa canal não-RC, sem iniciar navegação.

## Escritores, limites e encerramento

A verificação do grafo rejeita escritores conhecidos concorrentes. Complementa
a composição declarada; não é isolamento de segurança DDS contra publicadores
arbitrários.

O backend verifica validade e aplica limites. Zero longitudinal não demonstra
frenagem física; servo normalizado não mede ângulo da roda. Watchdog, RF,
sinais, centro, extremos e parada exigem ensaio. Um processo morto não pode
publicar seu próprio comando de parada.

## Ensaio direct_commands

`direct_commands` inicia somente o driver VESC e aceita comandos nos tópicos
finais. Não inclui RC, autoridade, backend ou seu lease; a configuração do
driver fornece porta e limites.

Consulte o [guia de veículo](../control/ebva_vehicle_control/README.md).
Não execute outra composição de atuação ao mesmo tempo.
