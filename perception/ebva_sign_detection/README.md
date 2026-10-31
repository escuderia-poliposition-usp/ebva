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

## Referências

[Arquitetura](../../docs/architecture.md) · [Contratos compartilhados](../../docs/interfaces.md) · [Contribuir](../../CONTRIBUTING.md) · [Licenças e atribuições](../../THIRD_PARTY_NOTICES.md)
