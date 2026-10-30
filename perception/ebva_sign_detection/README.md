# ebva_sign_detection — placas por YOLO

Yolo_node recebe imagens e publica vision_msgs/Detection2DArray e imagem anotada,
preservando o header original. Classe/score acompanham caixas em pixels;
o pacote não estima posição 3D.

~~~bash
ros2 launch ebva_sign_detection detector.launch.py params_file:=/dados/detector.yaml
~~~

O default é config/detector.yaml. Informe model_path com checkpoint .pt real
e selecione device (cpu ou dispositivo CUDA preparado). O pacote não distribui
pesos, instala drivers ou converte engines.

## Parâmetros

- model_path, device, imgsz, confidence: modelo/backend.
- input_topic e compressed_input: Image ou CompressedImage.
- detections_topic e annotated_topic: saídas.
- input_reliability e queue_depth: QoS/fila.

O default genérico usa /camera/image_raw; o YAML instalado referencia imagem
retificada da ZED. Ajuste tópicos/configuração no dono; o perfil só compõe.

O serviço /yolo/status expõe contagens, erros, tempos e metadados.
Qualidade depende de modelo, classes, dados e dispositivo. Testes determinísticos
não demonstram acurácia ou desempenho GPU.

## Build e testes

~~~bash
colcon build --packages-up-to ebva_sign_detection --symlink-install
source install/local_setup.bash
colcon test --packages-select ebva_sign_detection
colcon test-result --verbose
~~~

O backend neural precisa das bibliotecas do dispositivo selecionado.
O pacote mantém licença Apache-2.0.
