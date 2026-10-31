# Escrever e manter a documentação

A documentação deve permitir que alguém fora da equipe prepare o ambiente,
escolha uma composição e entenda os dados que cruzam cada fronteira. Escreva
em português e mantenha os identificadores exatamente como instalados.

## Onde documentar

| Local | Pergunta que responde |
| --- | --- |
| README principal | O que é o EBVA, o que contém e como começar? |
| README de pacote | Qual função implementa, como se organiza e como se conecta? |
| Referência em docs/ | Qual contrato ou procedimento vale nesta revisão do código? |
| Wiki | Como completar uma tarefa, que resultado esperar e como investigar falhas? |

Configuração permanece no dono. Notas pessoais, planos de execução, STATEs
e handoffs não fazem parte da documentação distribuída do produto.

## Padrão de README de pacote

Use estas seções nesta ordem, com conteúdo específico da função:

1. **Objetivo:** problema resolvido, papel na composição e limites funcionais.
2. **Organização:** diretórios reais, módulos/métodos, recursos e testes.
3. **Interfaces:** executáveis/launches ou API; tópicos/serviços, tipos,
   direção e nomes relativos/absolutos. Descreva unidades, frames, timestamp,
   QoS e perdas, ou vincule o contrato local que os define.
4. **Dependências:** pacotes próprios, ROS, bibliotecas e fornecedores;
   vincule package.xml e explicite SDK/modelo/mapa externos.
5. **Uso e configuração:** terminal/workspace, argumentos, YAML e efeitos,
   com exemplo concreto e pré-requisitos.
6. **Validação:** comandos, dados e resultado observável; diferencie build,
   teste unitário, inspeção, replay e ensaio físico.
7. **Referências:** contratos locais e guias compartilhados pertinentes.

Biblioteca pura documenta imports/argumentos/retornos; pacote de mensagens
documenta campos e consumidores. Não invente tópicos para preencher o modelo.
Informe quando não houver suíte própria.

## Padrão de tutorial na Wiki

Comece pelo objetivo e resultado observável. Declare pré-requisitos, diretório
de execução e recursos externos. Explique passos em sequência, resultado de
cada etapa e diagnóstico das falhas comuns. Indique próxima tarefa e referência
do pacote responsável.

Os exemplos distinguem inspeção, processamento de software, escrita de arquivos
e abertura de dispositivos. Valores de fixture não são calibração. Placeholders
como /dados/modelo.pt precisam ser substituídos por entradas reais.

Preserve os slugs existentes e navegação Home/_Sidebar. Links da Wiki para fontes
usam caminhos do repositório; links da referência local são relativos, úteis
offline. Publique referência nova e tutoriais dependentes de forma coordenada.

## Revisar uma mudança

Confira executáveis em setup.py/CMake, argumentos no launch e parâmetros no
nó/YAML. Compare package.xml com bibliotecas importadas. Verifique arquivos,
âncoras, links e sintaxe; não execute coletor/build só para conferir Markdown.
Registre quais exemplos foram inspecionados e quais realmente executados.

Mudança em CLI, mensagem, default ou efeito exige atualizar README do dono e
tutoriais consumidores. As fontes da estrutura estão nas
[referências editoriais](documentation-references.md).
