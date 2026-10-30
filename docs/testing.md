# Testar o EBVA

Use ROS Humble/Python 3.10 e as dependências dos pacotes. O ambiente nativo da
Jetson é a referência desta entrega. Docker no WSL e Gym têm preparação própria
no roadmap.

## Build e suítes

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=184
colcon test --base-paths src/ebva --executor sequential --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
~~~

O comando limita a descoberta dos testes aos pacotes do EBVA; fornecedores não
são testados automaticamente. O executor sequencial evita concorrência entre
suítes que usam players rosbag2, relógio ou serviços fixos.

Escolha domínio sem outra sessão. Para mudança local, selecione os pacotes
afetados; mudanças compartilhadas exigem consumidores. Confira XMLs, falhas,
erros e skips: retorno zero de colcon test sozinho não comprova aprovação.

Algumas suítes exigem recursos instalados ou ambiente específico. Leia o motivo
de cada skip e execute a verificação exigida quando fizer parte da mudança.
Ausência de dependência ou resultado não é aprovação.

## Gravação em instalação isolada

Dois casos de ebva_recording verificam os recursos e o ambiente de uma instalação
isolada. Execute-os em um workspace novo, somente com esse pacote sobre o
underlay /opt/ros/humble e sem user site-packages. CANDIDATES_INSTALLED=1
habilita esses casos depois que essa condição tiver sido preparada; a flag
não cria isolamento.

A partir da raiz do workspace que contém src/ebva:

~~~bash
recording_source="$PWD/src/ebva/recording/ebva_recording"
test_workspace="$(mktemp -d /tmp/ebva-recording-test.XXXXXX)"
env -i HOME="$HOME" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=184 \
  EBVA_RECORDING_SOURCE="$recording_source" EBVA_TEST_WORKSPACE="$test_workspace" \
  bash --noprofile --norc -e -o pipefail -c '
    source /opt/ros/humble/setup.bash
    cd "$EBVA_TEST_WORKSPACE"
    colcon build --base-paths "$EBVA_RECORDING_SOURCE" --symlink-install
    source install/local_setup.bash
    CANDIDATES_INSTALLED=1 colcon test --base-paths "$EBVA_RECORDING_SOURCE" --executor sequential --return-code-on-test-failure --event-handlers console_direct+
    colcon test-result --verbose
  '
~~~

Use domínio livre e execute após encerrar outras campanhas nesse domínio.
O build e os resultados ficam no diretório temporário informado; nenhum overlay
do workspace comum é carregado nesse processo.

## Campanhas

~~~bash
ros2 run ebva_system_tests run_all --output /tmp/ebva-campaigns-new
~~~

O destino deve ser novo. `--campaign manual` seleciona a campanha manual.
O resumo registra comandos e retornos. Campanhas usam dados sintéticos,
bags temporárias e relógio explícito; não abrem sensores ou atuadores.
Players com serviços fixos no mesmo domínio devem rodar serialmente.

Os perfis de teste usam componentes/contratos públicos. Conversões identificadas
como software_fixture não qualificam atuação física.

| Mudança | Verificação |
| --- | --- |
| Algoritmo | Casos numéricos, domínio e regressão |
| Interface | Tipo, unidade/frame, stamp, validade e consumidor |
| Supervisor | Seleção automática/fixa, regras, invalidez e expiração |
| Autoridade/backend | RC, neutro, perdas, desarme e ausência de rearme |
| Recurso/launch | Instalação e check_only |
| Ferramenta com saída | Entradas explícitas, destino e proteção de dados |
| Documentação | Links, paths, sintaxe e interface real |

Build, import, inspeção e replay respondem a perguntas diferentes. Check_only
não inicia stacks nem comprova mapas, modelos ou recursos externos.
Software não substitui calibração, RF, watchdog, frenagem ou precisão física.

Mantenha bags, logs completos, modelos e relatórios locais fora do Git.
Fixtures pequenas e oráculos numéricos necessários acompanham seus testes.
Informe ambiente, comando, resultado e limites na evidência da revisão.
