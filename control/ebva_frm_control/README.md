# ebva_frm_control

Implementações selecionadas por função, inicialmente experimentais.

Algoritmos e dialetos de comando são preservados por executável. Nenhum nó é iniciado pela instalação.

## Executáveis

- `point_sample`: núcleo pontual preservado, saída atômica DriveEffort com contrato frm_point_sample_v1 e validade do scan. Frame explícito obrigatório.
- `scan_average`

## Validação

Em workspace ROS Humble/Python 3.10 preparado: colcon build/test com este pacote e suas dependências. Fontes, CSVs/configurações e testes acompanham o pacote; Docker é opção do consumidor.

Não existe qualificação física ou autoridade RC comum implícita. Origem e licenças em [atribuições de terceiros](../../THIRD_PARTY_NOTICES.md).

Perfil: ros2 launch ebva_frm_control select.launch.py method:=METODO.
Métodos têm configurações/dialetos próprios; não compor múltiplos produtores
dos mesmos tópicos. Sensores/odometria/ponte de comando vêm da composição.
Não há ensaio físico. Scans inválidos, geometria do LiDAR e comportamento
de ausência de entrada exigem qualificação específica antes de operação.

FRM preserva amostra pontual vs média de dez raios e limites diferentes de
aceleração. Índices 180/360/540/720/900 assumem scan de 1080 raios; não usar
outra geometria. Sem ensaio de watchdog/perda de scan, não qualificado no carro.

## Contrato pontual adaptado no C2

[Lei longitudinal/lateral, geometria, validade e par atômico](docs/point-sample-contract.md).
A finalidade é seguimento reativo por diferenças de distâncias; saídas são
esforços adimensionais. Não são velocidade/aceleração ou yaw rate físicos.
O executável anterior autodrive foi substituído por point_sample, sem alias;
a origem bruta permanece preservada. scan_average mantém sua finalidade e ganhos.
A composição software fica no bringup e não inicia sensores ou atuadores.
