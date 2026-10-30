# Compatibilidade Humble — 29/10/2026

A imagem nativa exporta SmacPlannerHybrid, RPP e os plugins de costmap/behaviors selecionados.
Os dois plugins Navigator abaixo não são exportados pelo Humble instalado. As declarações de versão posterior foram retiradas do default ativo; demais parâmetros numéricos foram preservados.

~~~yaml
navigators:
- navigate_to_pose
- navigate_through_poses
navigate_to_pose:
  plugin: nav2_bt_navigator::NavigateToPoseNavigator
navigate_through_poses:
  plugin: nav2_bt_navigator::NavigateThroughPosesNavigator
~~~

Validação somente de recursos e identificadores; servidores não foram ativados.
