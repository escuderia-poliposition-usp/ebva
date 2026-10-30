# Licenças e atribuições

O [LICENSE](LICENSE) aplica MIT ao código de autoria da Escuderia Poliposition USP.
Avisos específicos das fontes externas são preservados e continuam aplicáveis
aos respectivos componentes.

| Componente | Declaração preservada | Texto |
| --- | --- | --- |
| Fontes F1TENTH extraídas para sensores, localização, diagnóstico, gravação e bringup | MIT; F1TENTH Autonomous Racing Community, 2024 | [MIT-F1TENTH](LICENSES/MIT-F1TENTH.txt) |
| ebva_sign_detection | Apache-2.0 | [Apache-2.0](LICENSES/Apache-2.0.txt) |
| Waypoint Pure Pursuit de escuderia_pure_pursuit_pkg em ebva_path_control | Apache-2.0 | [Apache-2.0](LICENSES/Apache-2.0.txt) |
| Ponte, configuração e teleop AutoDRIVE | BSD-2-Clause; Tinker Twins, 2023/2024 | [BSD-2-Clause](LICENSES/BSD-2-Clause-Tinker-Twins.txt) |

Referências de origem: [F1TENTH](https://github.com/f1tenth/f1tenth_system),
[AutoDRIVE da equipe](https://github.com/escuderia-poliposition-usp/EBVA_AutoDRIVE_SimRacing).
Os arquivos que possuem avisos de copyright mantêm seus avisos.
Movimentos não atribuem autoria da equipe ao código externo.

## Controle de trajetória

A declaração Apache-2.0 de escuderia_pure_pursuit_pkg acompanha a matemática de
waypoint Pure Pursuit conservada em ebva_path_control. As implementações estão
em ebva_path_control/waypoint_pp/waypoint_follow.py e
ebva_path_control/waypoint_curvature.py. O texto Apache permanece em
[LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt); os avisos individuais dos
arquivos continuam aplicáveis.

Os contratos, wrappers e adaptações novos de autoria da equipe usam MIT.
A composição do pacote declara MIT AND Apache-2.0, sem substituir a licença da
matemática de origem.

## Identificações das fontes da equipe

As fontes de calibração identificavam jooj como mantenedor; as de controle por
faixas, cristian; métodos de controle também identificavam lca e poliposition.
Essas identificações de manutenção são preservadas como crédito, sem tratá-las
como declaração individual de autoria ou licença.

Fornecedores em src/external são repositórios independentes com licenças
próprias. Dependencies.repos registra URLs/revisões sem incorporar seus Gits ao
EBVA. Bibliotecas ROS, SDK ZED, modelos neurais e simuladores externos possuem
termos próprios.

Ao acrescentar fonte externa, registre origem, licença e os avisos necessários.
Um campo TODO não constitui declaração de licença.
