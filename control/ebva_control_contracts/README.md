# ebva_control_contracts

Contratos puros de validade temporal para controladores/adaptadores EBVA,
sem ROS I/O ou dependência da integração. MeasurementLease conserva idade ROS
original e deadline monotônico; duplicatas/reordenação não renovam o lease.
Extraído sem alteração de métodos de lane keeping no C3, com consumidores reais
lane keeping, FTG/PD e adaptador FTG/PD. Sem conversões/calibração de unidades.
