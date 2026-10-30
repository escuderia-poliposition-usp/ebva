# Adaptador de esforço longitudinal e ângulo

LongitudinalEffortSteering representa esforço longitudinal adimensional e
ângulo geométrico rad, header original e ID de contrato de uma lei identificada.
Nenhum ganho de velocidade/aceleração é inferido do throttle. O adaptador
aceita exatamente um contrato configurado, frame original, stamp canônico,
idade200ms, futuro20ms e ordem estrita. Nunca renova header ou repete comando;
o heartbeat é somente diagnóstico. Autoridade RC e backend expiram separadamente.

EffortSteeringMapping só opera com mapping_purpose=software_fixture,
calibration_id explícito, ganhos/domínios/limites finitos. Fora do domínio de
entrada é rejeitado; limites elétricos saturam somente uma entrada válida.
Na fixture:100eRPM/unidade e-0,2servo/rad, centro0,5,±600eRPM e servo0,4–0,6.
Isso não é calibração medida, frenagem ou comando físico homologado.

RC seleciona motor/direção integralmente e tem prioridade sobre toda autonomia.
Retorno ao neutro depende de entradas válidas. Sem aquisição/driver/dispositivos
nos perfis de replay; seus comandos permanecem no domínio ROS de software.
