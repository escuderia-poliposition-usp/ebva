# Métodos de faixas

road_mask_v4_80, road_mask_v4_50 e color_gradient_v3 compartilham a geometria
do pipeline e mantêm binarizações distintas. Os limiares v4 são escolhas
explícitas de método, não uma classificação de superioridade.

## Entrada e saída reais

Entrada sensor_msgs/Image em image_topic, best_effort. Conversão explícita por
cv_bridge para BGR8 aceita bgr8/rgb8/mono8/bgra8/rgba8. Encodings16bit/float/Bayer
não recebem escala/conversão presumida. Envelope deve ter width/height≥2,
step suficiente, dados exatos height×step, frame não vazio e stamp canônico
positivo (sec≥0,0≤nanosec<1e9). Não se infere frescor da recepção DDS.

Saída principal /lane_detection/lane_boundaries é **lane_msgs/LaneBoundaries**,
Header+Point[] left_lane/right_lane, z=0. x para frente, y para direita e origem
na câmera; metros calculados pelos parâmetros/homografia assumidos, sem prova
física. header.stamp é cópia exata da imagem. header.frame_id é camera_frame_id
(default histórico camera_link), que precisa designar esse plano xfrente/ydireita.
Isso não aplica TF nem converte automaticamente um frame óptico ou FLU.

Lado escolhido pelo sinal lateral da base do rastro. Pontos seguem a ordem do
crawler desde a base; curvas fechadas não garantem x globalmente crescente.
Vazio significa ausência de faixa naquele quadro. LaneTracker continua disponível
para pesquisa offline, mas o wrapper permanece sem tracker/coasting: geometria
antiga não recebe stamp do quadro novo. Não há publicação periódica durante
silêncio. O produtor não rejeita por idade/reordenação, e nunca renova esses
stamps; o consumidor lane_keeping mantém seus guards de idade/frame/novidade e
conversões próprios, 

/lane_detection/birdseye_debug tem nome histórico; publica overlay na imagem
original BGR8, mesmas dimensões e **header original da imagem de entrada**.
O overlay não muda referencial dos pixels. O replay usa nomes distintos de entrada óptica e saída métrica para verificar
que o overlay conserva o frame da imagem, independente da segmentação.
/lane_detection/status (std_msgs/String JSON) relata uma decisão por entrada,
método, stamp/frame original, frame declarado e flags sem coasting/calibração.
É diagnóstico, não fonte de comando. Parâmetros são lidos na construção; ros2
param set posterior não reconfigura cálculo. Método inválido/geometria degenerada
falha antes da inscrição; cleanup da construção evita nó parcial.

## Comparação e testes

Compare os métodos sobre os mesmos quadros, mantendo parâmetros geométricos
coerentes. Diferenças na quantidade de pixels ou pontos não demonstram qualidade
sem revisão ou ground truth. Uma cena sintética gerada com a mesma homografia
da reconstrução não valida extrínsecos independentes.

A campanha lane_perception_replay, em ebva_system_tests, exercita BGR/RGB/mono,
rejeição de encoding/step/stamp/frame/dimensões, faixa vazia, silêncio e
recuperação. Verifica coordenadas, headers, z zero e debug, sem tratar a
geometria calculada como ground truth.

O wrapper não publica comandos de atuação. Homografia, escala, corpo e montagem
da câmera precisam ser avaliados antes de conectar um controlador no veículo.
