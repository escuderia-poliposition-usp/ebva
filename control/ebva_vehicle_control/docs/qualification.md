# Verificar a cadeia de controle

## Escopos de verificação

| Nível | O que demonstra |
| --- | --- |
| Testes puros | Parser, políticas, conversões, domínio e limites matemáticos |
| Testes ROS/DDS | Tipos, grafos, serviços, validade, concorrência e ciclo de vida |
| Replay sintético | Integração de um produtor com adaptador, autoridade e backend |
| Ensaio físico | Comportamento do conjunto sensor/RC/driver/firmware/atuador |

A suíte de software utiliza mensagens sintéticas sem dispositivos. Sua
aprovação não mede RF, watchdog de firmware, frenagem, centro ou extremos
reais da direção.

## Adaptadores específicos

Ackermann, FRM, FTG/PD, esforço/ângulo, modelos CBF e supervisor têm semânticas
próprias. Os adaptadores software_fixture exigem propósito, calibração sintética,
frame, ganhos e limites explícitos. Não constituem uma conversão longitudinal
universal.

As verificações temporais preservam stamps originais, rejeitam entradas inválidas
e impedem renovação de medidas antigas. Quando há várias entradas, a saída
conserva a validade da mais antiga. A seleção RC acontece depois da conversão
autônoma, cobrindo o par motor/direção.

## Casos da autoridade/backend

- RC válido prevalece; neutro manual aplica zero e centro configurado.
- Com autonomia habilitada, neutro retoma somente entradas válidas.
- Perda/duplicidade de RC bloqueia seleção; autonomia duplicada não deve
  invalidar um RC manual válido.
- Concorrência nos comandos finais desarma e exige habilitação posterior.
- Expiração da seleção é observada independentemente da aquisição.
- Reiniciar autoridade não rearma o backend.
- Shutdown e falhas de callbacks preservam limpeza e propagação de erros.

Partida automática é uma escolha do YAML manual, aguardando condições válidas
uma vez. O teste desse comportamento usa grafo e RC sintéticos.

## Ensaios do veículo

Antes do ensaio, identifique dispositivos, firmware, operador, alimentação,
montagem e a configuração efetiva. Meça polaridades, resposta em baixa rotação,
centro/extremos e arcos de direção. Observe o comportamento após perda do RC,
aquisição, autoridade, backend e driver.

O protocolo serial de dois canais informa pulsos e validade local; não comprova
RF válido quando um receptor repete pulsos. Comando/eco do servo não mede ângulo
real. Longitudinal zero não demonstra frenagem.

Valores de fixtures nunca devem ser promovidos automaticamente à configuração
do veículo. O [contrato de autoridade](../../../docs/command-authority.md)
e o [guia de testes](../../../docs/testing.md) descrevem os requisitos comuns.
