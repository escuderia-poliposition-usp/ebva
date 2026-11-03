# ebva_sign_detection — detecção de placas por YOLO

## Objetivo

Detectar placas em imagens ROS com inferência CPU/CUDA e publicar caixas, classes, scores e imagem anotada, preservando o header original. A função é detecção 2D; profundidade tem outro dono.

## Organização

| Caminho | Conteúdo |
| --- | --- |
| [ebva_sign_detection/](ebva_sign_detection/) | Módulos Python, algoritmos e adaptadores da função. |
| [config/](config/) | Defaults, parâmetros e fixtures de configuração. |
| [launch/](launch/) | Entradas ROS launch para seleção/composição. |
| [test/](test/) | Verificações e fixtures locais. |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Dependências e instalação de recursos/entry points. |

Pontos de entrada do código:

- [yolo_node.py](ebva_sign_detection/yolo_node.py): mensagens, QoS e status.
- [backend.py](ebva_sign_detection/backend.py): carregamento/inferência PyTorch e Ultralytics.

## Interfaces

`input_topic`: `Image` ou `CompressedImage`. `detections_topic`: `Detection2DArray` (default `/yolo/detections`); `annotated_topic`: `Image` (default `/yolo/annotated_image`). `/yolo/status`: `std_srvs/Trigger`, com contagens/metadados JSON. `detector.launch.py` recebe `params_file`.

## Dependências

A declaração canônica está em [package.xml](package.xml).

- **Pacotes do EBVA:** nenhuma dependência direta declarada.
- **ROS e bibliotecas externas:** `rclpy`, `sensor_msgs`, `vision_msgs`, `std_srvs`, `cv_bridge`, `python3-numpy`, `launch`, `launch_ros`, `ament_index_python`.
- **Build:** `ament_python`. **Testes:** `ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`.

A inferência também exige **PyTorch e Ultralytics** no Python do nó: são imports do backend, não instalados pelo colcon. Checkpoint `.pt` é externo; `device` aceita `cpu` ou `cuda:0`. CUDA requer runtime compatível com host/JetPack e não faz fallback silencioso para CPU.

## Uso e configuração

~~~bash
ros2 launch ebva_sign_detection detector.launch.py params_file:=/dados/detector.yaml
~~~

O default é config/detector.yaml. Informe model_path com checkpoint .pt real
e selecione device (cpu ou dispositivo CUDA preparado). O pacote não distribui
pesos, instala drivers ou converte engines.

### Parâmetros

- model_path, device, imgsz, confidence: modelo/backend.
- input_topic e compressed_input: Image ou CompressedImage.
- detections_topic e annotated_topic: saídas.
- input_reliability e queue_depth: QoS/fila.

O default genérico usa /camera/image_raw; o YAML instalado referencia imagem
retificada da ZED. Ajuste tópicos/configuração no dono; o perfil só compõe.

O serviço /yolo/status expõe contagens, erros, tempos e metadados.
Qualidade depende de modelo, classes, dados e dispositivo. Testes determinísticos
não demonstram acurácia ou desempenho GPU.

## Validação

A partir da raiz de um workspace preparado com ROS Humble e dependências instaladas:

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ebva_sign_detection
source install/local_setup.bash
colcon test --packages-select ebva_sign_detection --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

O backend neural precisa das bibliotecas do dispositivo selecionado.
O pacote mantém licença Apache-2.0.

Consulte [o guia de testes](../../docs/testing.md) para domínio isolado, campanhas e requisitos especiais. Registre falhas, erros e skips. Software não qualifica sensores, calibração ou atuação física.

## Fontes externas de modelos

O pacote consome checkpoints de detecção PyTorch (.pt), sem importar código do
projeto de treino. Para a campanha RoboRace 2026, notebooks e procedência ficam
no repositório privado
[traffic-perception-training](https://github.com/escuderia-poliposition-usp/traffic-perception-training).
O acesso ao repositório de treino não é necessário em runtime.

A fonte treinada deve entregar os arquivos abaixo fora do Git do produto:

| Arquivo | Conteúdo |
| --- | --- |
| best.pt | Checkpoint de detecção selecionado pela validação |
| inference.yaml | Parâmetros ROS model_path, imgsz, confidence e device |
| README.md e relatório | Origem, versão do dataset, inicialização e resultados de avaliação |

O exportador do projeto de treino produz esse pacote. Ajuste model_path ao
armazenamento do dispositivo e complete tópicos/QoS para o ambiente. A escolha
de modelo e confiança precede a avaliação do teste separado por cenário.
Não substituir defaults por um checkpoint de teste de software.

Os nomes são metadados do checkpoint; o backend não remapeia classes para
compensar um modelo incompatível. Confira-os em /yolo/status antes do replay.
Instale Torch compatível com o dispositivo/JetPack; não copie ambientes Python
ou engines de outra arquitetura para consumir o .pt.

A campanha usa IDs 0–8 nesta ordem: go_straight, green_light, no_left_turn,
no_right_turn, no_straight, red_light, turn_left, turn_right, yellow_light.
Conferir nomes em /yolo/status e headers de entrada/saída em replay isolado
antes de selecionar a fonte no veículo. O nó publica detecções por imagem;
confirmação temporal e decisão de condução têm avaliação e integração próprias.

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
