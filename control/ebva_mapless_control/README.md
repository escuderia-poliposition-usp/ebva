# ebva_mapless_control

Implementações selecionadas por função, inicialmente experimentais.

Algoritmos e dialetos de comando são preservados por executável. Nenhum nó é iniciado pela instalação.

## Executáveis

- `scan_direct`
- `ftg_pd`: PD real, duas entradas estampadas e par FtgPdCommand.
- `scan_pd`

## Validação

Em workspace ROS Humble/Python 3.10 preparado: colcon build/test com este pacote e suas dependências. Fontes, CSVs/configurações e testes acompanham o pacote; Docker é opção do consumidor.

Não existe qualificação física ou autoridade RC comum implícita. Origem e licenças em [atribuições de terceiros](../../THIRD_PARTY_NOTICES.md).

Perfil: ros2 launch ebva_mapless_control select.launch.py method:=METODO.
Métodos têm configurações/dialetos próprios; não compor múltiplos produtores
dos mesmos tópicos. Sensores/odometria/ponte de comando vêm da composição.
Não há ensaio físico. Scans inválidos, geometria do LiDAR e comportamento
de ausência de entrada exigem qualificação específica antes de operação.

scan_direct: TTC10, throttle máximo .24, /scan, direção direta.
ftg_pd: TTC30, throttle .12, esforço PD com scan/JointState sintéticos no perfil C3.
scan_pd: TTC50, throttle .12, /scan, direção direta (o PD está comentado na fonte). Não são defaults intercambiáveis.

## C3 — contrato FTG/PD

[Saídas, feedback, tempo e correção dirigida](docs/ftg-pd-contract.md).
ftg_pd substitui o executável autodrive_pd, sem alias. Os dois métodos de direção
direta permanecem separados, com arquivos/parâmetros preservados. A saída PD não
é ângulo final ou esforço normalizado. Feedback físico/AutoDRIVE ainda precisa
contrato/medição próprios; a composição C3 usa somente fixture explícita.
