# ebva_sign_depth — segmentação clássica e profundidade

Sign_depth segmenta placas circulares vermelhas e compara profundidade média
da região com estimativa por diâmetro/focal. Publica debug/logs; não substitui
YOLO nem publica mensagem estruturada de detecção/distância.

~~~bash
ros2 run ebva_sign_depth sign_depth
~~~

RGB/profundidade registrada devem estar alinhadas e com mesma resolução.
Parâmetros incluem topico_entrada, topico_profundidade, topico_camera_info,
diametro_real_m, slop_sincronizacao, limiares e publicar_debug.

Profundidade é Z da câmera: 32FC1 em metros ou 16UC1 em milímetros convertido
para metros. Encodings desconhecidos ou dimensões incompatíveis são rejeitados.
Diâmetro e intrínsecos devem corresponder ao objeto e câmera reais.

~~~bash
colcon build --packages-up-to ebva_sign_depth --symlink-install
colcon test --packages-select ebva_sign_depth
colcon test-result --verbose
~~~

Fixtures verificam geometria, unidades e segmentação. Precisão nas imagens
do veículo exige dados e avaliação próprios.
