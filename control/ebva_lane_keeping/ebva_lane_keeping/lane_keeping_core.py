"""
Núcleo del lane keeping SIN ROS (v2, 2026-10-22).

Toda la lógica de decisión vive aquí para poder probarla offline (simulación y tests)
con exactamente el mismo código que corre en el carro. lane_keeping_node.py solo
traduce mensajes ROS <-> este núcleo.

Pipeline por cada mensaje de Francisco:
    puntos -> Adapter (geometría, 1 o 2 líneas) -> FSM (confianza)
           -> filtro de error -> Stanley (dirección) -> gestor de velocidad

Mejoras respecto a la v1 (cada una se puede apagar con su parámetro):
  * single_line_fallback  : sigue con una sola línea visible (curvas, marcas gastadas).
  * error_filter_alpha    : suaviza lateral/heading/curvatura (1.0 = sin filtro).
  * max_lateral_accel     : baja la velocidad en curvas cerradas: v <= sqrt(a_lat / |k|).
  * accel_limit           : rampa de aceleración al arrancar (evita tirones/patinadas).
  * single_line_speed_factor : más lento cuando se ve una sola línea.
  * invalid_hold_time     : un cuadro malo aislado ya no frena el carro en seco; se mantiene
                            el último giro a velocidad reducida hasta invalid_hold_time
                            (la FSM sigue parando en LOST como antes).
  * velocidad medida      : Stanley usa la velocidad real si llega odometría fresca;
                            si no, la velocidad comandada (rampa), nunca una constante fija.
"""

import math

from .confidence_fsm import ConfidenceFSM, LaneState
from .lane_boundaries_adapter import LaneBoundariesAdapter
from .stanley_controller import StanleyController

STATE_CODE = {LaneState.TRACKING: 1.0, LaneState.DEGRADED: 2.0, LaneState.LOST: 3.0}
MODE_CODE = {'none': 0.0, 'both': 2.0, 'left': -1.0, 'right': 1.0}


DEFAULTS = dict(
    # --- velocidad ---
    vehicle_speed=0.8,
    degraded_speed_factor=0.6,
    single_line_speed_factor=0.8,
    max_lateral_accel=2.0,        # m/s^2; 0 = sin límite por curvatura
    min_moving_speed=0.0,         # m/s; piso cuando se mueve (para salir de la zona de tirones)
    accel_limit=1.0,              # m/s^2; 0 = sin rampa
    # --- Stanley ---
    k_stanley=3.0,                # sim: k=1 deja ~20 cm de error en curvas; k=3 ~3 cm
    k_soft=0.5,
    max_steering=0.26,            # límite real del servo según vesc.yaml
    feedforward_gain=0.5,         # la parábola sobreestima la curvatura; 1.0 corta la curva
    wheelbase=0.33,
    steering_rate=3.0,
    # --- FSM ---
    degraded_after=0.15,
    lost_after=0.50,
    recovery_time=0.20,
    # --- seguridad ---
    stop_on_invalid=True,
    invalid_hold_time=0.25,       # s; huecos cortos de detección: mantener último comando (lento)
    latch_lost=True,
    # --- adaptador ---
    lookahead_x=0.0,              # medir el error en el carro (Stanley); 0.5 lo cuenta doble en curvas
    single_line_fallback=True,
    lane_width_init=0.50,
    weighted_fit=True,
    # --- filtro ---
    error_filter_alpha=0.7,       # peso del dato nuevo; 1.0 = sin filtro
    measured_speed_timeout=0.3,   # s; odometría más vieja que esto se ignora
)

# Parámetros que reproducen la v1 (para comparar o volver atrás sin tocar código).
V1_COMPAT = dict(
    lookahead_x=0.5,
    k_stanley=1.0,
    feedforward_gain=1.0,
    max_steering=0.45,
    single_line_fallback=False,
    weighted_fit=False,
    error_filter_alpha=1.0,
    max_lateral_accel=0.0,
    accel_limit=0.0,
    single_line_speed_factor=1.0,
    min_moving_speed=0.0,
    invalid_hold_time=0.0,
)


class LaneKeepingCore:

    def __init__(self, **params):
        p = dict(DEFAULTS)
        unknown = set(params) - set(p)
        if unknown:
            raise ValueError(f'Parámetros desconocidos: {sorted(unknown)}')
        p.update(params)
        self.p = p

        self.adapter = LaneBoundariesAdapter(
            lookahead_x=p['lookahead_x'],
            min_points=3,
            min_lane_width=0.15,
            max_lane_width=1.20,
            max_abs_slope=3.0,
            max_abs_curvature=5.0,
            max_abs_lateral_error=1.0,
            single_line_fallback=p['single_line_fallback'],
            lane_width_init=p['lane_width_init'],
            weighted_fit=p['weighted_fit'],
        )
        self.controller = StanleyController(
            k_stanley=p['k_stanley'],
            max_steering=p['max_steering'],
            feedforward_gain=p['feedforward_gain'],
            wheelbase=p['wheelbase'],
            steering_rate=p['steering_rate'],
            k_soft=p['k_soft'],
        )
        self.fsm = ConfidenceFSM(
            degraded_after=p['degraded_after'],
            lost_after=p['lost_after'],
            recovery_time=p['recovery_time'],
        )
        self.reset()

    # ------------------------------------------------------------------

    def reset(self):
        """Estado inicial: parado, FSM en LOST, sin enclavamiento."""
        self.fsm.state = LaneState.LOST
        self.fsm.valid_time = 0.0
        self.fsm.invalid_time = 0.0
        self.controller.reset()
        self.adapter.reset()
        self.has_tracked = False
        self.latched = False
        self.speed_cmd = 0.0
        self.filt = None
        self.measured_speed = None
        self.measured_age = float('inf')
        self.invalid_elapsed = 0.0
        self.last_valid_steering = None

    def set_measured_speed(self, speed):
        if speed is not None and math.isfinite(speed):
            self.measured_speed = float(speed)
            self.measured_age = 0.0

    def force_stop(self):
        """Watchdog / timeout: parar y, si ya se seguía el carril y latch_lost, enclavar.
        La FSM vuelve a LOST: al volver el detector se exige recovery_time de detección
        estable antes de moverse (la v1 podía rearrancar al instante)."""
        if self.p['latch_lost'] and self.has_tracked:
            self.latched = True
        self.fsm.state = LaneState.LOST
        self.fsm.valid_time = 0.0
        self.controller.reset()
        self.speed_cmd = 0.0
        self.filt = None
        self.last_valid_steering = None

    # ------------------------------------------------------------------

    def _filter(self, lateral, heading, curvature):
        a = self.p['error_filter_alpha']
        new = (lateral, heading, curvature)
        if self.filt is None or a >= 1.0:
            self.filt = new
        else:
            self.filt = tuple(a * n + (1.0 - a) * f for n, f in zip(new, self.filt))
        return self.filt

    def _target_speed(self, state, mode, curvature):
        p = self.p
        v = p['vehicle_speed']
        if state == LaneState.DEGRADED:
            v *= p['degraded_speed_factor']
        if mode in ('left', 'right'):
            v *= p['single_line_speed_factor']
        if p['max_lateral_accel'] > 0.0 and abs(curvature) > 1e-3:
            v = min(v, math.sqrt(p['max_lateral_accel'] / abs(curvature)))
        if v > 0.0 and p['min_moving_speed'] > 0.0:
            v = max(v, min(p['min_moving_speed'], p['vehicle_speed']))
        return v

    def _ramp(self, target, dt):
        lim = self.p['accel_limit']
        if lim <= 0.0 or target <= self.speed_cmd:
            self.speed_cmd = target          # frenar es inmediato
        else:
            self.speed_cmd = min(target, self.speed_cmd + lim * max(dt, 0.0))
        return self.speed_cmd

    def _speed_for_stanley(self):
        if (self.measured_speed is not None
                and self.measured_age <= self.p['measured_speed_timeout']):
            return abs(self.measured_speed)
        return self.speed_cmd

    # ------------------------------------------------------------------

    def step(self, left_lane, right_lane, dt):
        """Procesa un mensaje de carriles. Devuelve un dict con steering, speed y debug."""
        self.measured_age += max(dt, 0.0)

        out = dict(steering=0.0, speed=0.0, state=LaneState.LOST, detected=False,
                   mode='none', lateral=0.0, heading=0.0, curvature=0.0,
                   latched=self.latched, event=None)

        if self.latched:
            self.speed_cmd = 0.0
            out['event'] = 'latched'
            return out

        detected, lateral, heading, curvature, mode = self.adapter.compute_ex(left_lane, right_lane)
        out.update(detected=detected, mode=mode)

        dt_fsm = min(dt, 0.1) if detected else dt
        state = self.fsm.update(detected, dt_fsm)
        out['state'] = state

        if state == LaneState.TRACKING:
            self.has_tracked = True

        if state == LaneState.LOST:
            if self.p['latch_lost'] and self.has_tracked:
                self.latched = True
                out['event'] = 'latch'
            else:
                out['event'] = 'lost'
            self.controller.reset()
            self.speed_cmd = 0.0
            self.filt = None
            self.last_valid_steering = None
            out['latched'] = self.latched
            return out

        if not detected:
            out['event'] = 'invalid'
            self.invalid_elapsed += max(dt, 0.0)
            if (self.last_valid_steering is not None
                    and self.invalid_elapsed <= self.p['invalid_hold_time']):
                # Hueco corto: seguir con el último giro, a velocidad de DEGRADED.
                hold_speed = min(self.speed_cmd,
                                 self.p['vehicle_speed'] * self.p['degraded_speed_factor'])
                self.speed_cmd = hold_speed
                out.update(steering=self.last_valid_steering, speed=hold_speed, event='hold')
                return out
            if self.p['stop_on_invalid']:
                self.speed_cmd = 0.0
                out['steering'] = self.controller.previous_steering
                return out
            out['steering'] = None   # None = no publicar (comportamiento v1 con stop_on_invalid=False)
            return out

        self.invalid_elapsed = 0.0
        lateral, heading, curvature = self._filter(lateral, heading, curvature)

        steering = self.controller.compute_steering(
            lateral, heading, self._speed_for_stanley(),
            curvature=curvature, dt=dt,
        )
        speed = self._ramp(self._target_speed(state, mode, curvature), dt)
        self.last_valid_steering = steering

        out.update(steering=steering, speed=speed, lateral=lateral,
                   heading=heading, curvature=curvature)
        return out
