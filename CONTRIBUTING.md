# Contribuir com o EBVA

Alterações devem preservar o significado das interfaces e permitir que cada
pacote seja construído com suas dependências ROS/Python declaradas.

## Organização

- Implemente a função no pacote responsável; configurações e recursos acompanham
  o dono.
- Mantenha o bringup como orquestrador. Perfis listam componentes e referências
  instaladas, sem copiar parâmetros ou algoritmos.
- Preserve alternativas com propósito distinto, identificando entradas,
  saídas, unidades e requisitos. Perfis selecionam a execução.
- Separe algoritmos de I/O quando isso permitir testes determinísticos.
  Campanhas ficam em ebva_system_tests e fontes sintéticas em ebva_simulation;
  aplicação não deve depender deles para executar.
- Mantenha dados, modelos, resultados, builds e credenciais fora do Git.

## Interfaces

Declare tipo, unidade, sinal, frame/ponto do modelo, origem do timestamp,
QoS e expiração. Atualize produtor, consumidor, configuração, documentação
e testes da ligação. Alterar frame exige transformar os valores.

Comandos para o veículo passam pela autoridade/backend da composição,
preservando prioridade RC e limites. Interfaces de ensaio direto permanecem
explícitas e separadas.

## Validação

Use o [guia de testes](docs/testing.md). Registre no pull request os comandos,
ambiente, resultados e limites. Mensagens ou dependências compartilhadas exigem
build/testes dos consumidores afetados. Build, inspeção, replay e ensaio físico
são resultados diferentes.

Mudanças de fórmulas, controle ou calibração precisam de justificativa e evidência
próprias. Movimentos não devem alterar numericamente o método sem isso aparecer
na revisão.

## Revisão

Revise arquivos e hunks antes de adicionar ao índice. Mantenha commits com
propósito claro, explicando o comportamento resultante. A branch principal é
`master`.

A documentação técnica acompanha o código; a Wiki contém tutoriais com links
para a referência. Notas pessoais de trabalho ficam fora da documentação
distribuída.

O [padrão documental](docs/documentation.md) define as seções dos READMEs e a
estrutura dos tutoriais. Ao alterar interface, argumento ou configuração,
atualize a documentação do dono e os exemplos da Wiki que o utilizam.

Preserve licenças e avisos de fontes externas. Consulte
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) e declare novas dependências
no manifesto do pacote.
