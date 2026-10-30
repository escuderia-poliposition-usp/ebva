"""
Adaptador geométrico (v2):
LaneBoundaries (Francisco) -> lateral_error, heading_error, curvature

Frame de Francisco:
    x = adelante (m)
    y = derecha (+) / izquierda (-)
    frame_id = camera_link

Convención Stanley:
    lateral_error > 0 -> vehículo a la IZQUIERDA del centro del carril
    curvature > 0 -> giro a la IZQUIERDA

Cambios v2 (2026-10-22), todos compatibles con la interfaz anterior:
  1. Modo de UNA línea (single_line_fallback=True): si solo se ve una línea (típico en curvas
     o cuando una marca está gastada), el centro se estima desplazando esa línea medio
     ancho de carril. El ancho se aprende en vivo (EMA) mientras se ven las dos líneas.
     Antes, perder una línea = detección inválida = el carro se detenía.
  2. Ajuste ponderado: los puntos cercanos pesan más que los lejanos (los lejanos tienen
     más error por la perspectiva de la vista de pájaro).
  3. Si hay pocos puntos o poco alcance hacia adelante, el ajuste baja a recta (grado 1)
     en lugar de inventar curvatura con una parábola mal condicionada.
  4. compute() devuelve lo mismo que antes; compute_ex() además devuelve el modo
     ('both' | 'left' | 'right' | 'none') para el debug y la gestión de velocidad.

Con single_line_fallback=False y weighted_fit=False se comporta como la versión anterior.
"""

import math

import numpy as np


class LaneBoundariesAdapter:

    def __init__(
        self,
        lookahead_x=0.5,
        min_points=3,
        min_lane_width=0.15,
        max_lane_width=1.20,
        max_abs_slope=3.0,
        max_abs_curvature=5.0,
        max_abs_lateral_error=1.0,
        lateral_offset=0.0,
        single_line_fallback=True,
        lane_width_init=0.50,
        lane_width_alpha=0.10,
        min_points_single=4,
        weighted_fit=True,
        min_span_for_quadratic=0.40,
    ):
        self.lookahead_x = float(lookahead_x)
        self.min_points = int(min_points)

        self.min_lane_width = float(min_lane_width)
        self.max_lane_width = float(max_lane_width)
        self.max_abs_slope = float(max_abs_slope)
        self.max_abs_curvature = float(max_abs_curvature)
        self.max_abs_lateral_error = float(max_abs_lateral_error)

        self.lateral_offset = float(lateral_offset)

        self.single_line_fallback = bool(single_line_fallback)
        self.lane_width_init = float(lane_width_init)
        self.lane_width_est = float(lane_width_init)
        self.lane_width_alpha = float(lane_width_alpha)
        self.min_points_single = int(min_points_single)
        self.weighted_fit = bool(weighted_fit)
        self.min_span_for_quadratic = float(min_span_for_quadratic)

        self.last_mode = 'none'

    def reset(self):
        self.lane_width_est = self.lane_width_init
        self.last_mode = 'none'

    # ------------------------------------------------------------------
    # Ajuste de cada línea
    # ------------------------------------------------------------------

    def _clean_points(self, points):
        cleaned = []
        seen = set()

        for p in points:
            x = float(p.x)
            y = float(p.y)

            if not (math.isfinite(x) and math.isfinite(y)):
                continue

            if x <= 0.0:
                continue

            key = (round(x, 4), round(y, 4))

            if key in seen:
                continue

            seen.add(key)
            cleaned.append((x, y))

        return cleaned

    def _fit_side(self, points):
        """Devuelve (coef_grado2, n_puntos) o (None, n)."""
        cleaned = self._clean_points(points)
        n = len(cleaned)

        if n < self.min_points:
            return None, n

        xs = np.fromiter((p[0] for p in cleaned), dtype=np.float64, count=n)
        ys = np.fromiter((p[1] for p in cleaned), dtype=np.float64, count=n)

        span = float(xs.max() - xs.min())
        if span < 1e-3:
            return None, n

        weights = None
        if self.weighted_fit:
            # Peso ~ 1/x: el punto a 0.3 m pesa ~5x más que uno a 1.5 m.
            weights = 1.0 / np.maximum(xs, 0.1)

        degree = 2 if (n >= 4 and span >= self.min_span_for_quadratic) else 1

        try:
            coeff = np.polyfit(xs, ys, degree, w=weights)
        except (np.linalg.LinAlgError, ValueError, TypeError):
            return None, n

        if not np.all(np.isfinite(coeff)):
            return None, n

        if degree == 1:
            coeff = np.array([0.0, coeff[0], coeff[1]])

        return coeff, n

    # ------------------------------------------------------------------
    # Interfaz pública
    # ------------------------------------------------------------------

    def compute(self, left_lane, right_lane):
        detected, lateral, heading, curvature, _ = self.compute_ex(left_lane, right_lane)
        return detected, lateral, heading, curvature

    def compute_ex(self, left_lane, right_lane):
        invalid = (False, 0.0, 0.0, 0.0, 'none')

        left_coeff, n_left = self._fit_side(left_lane)
        right_coeff, n_right = self._fit_side(right_lane)

        x_eval = self.lookahead_x
        center_coeff = None
        mode = 'none'

        if left_coeff is not None and right_coeff is not None:
            left_y = float(np.polyval(left_coeff, x_eval))
            right_y = float(np.polyval(right_coeff, x_eval))
            width = right_y - left_y

            if self.min_lane_width <= width <= self.max_lane_width:
                center_coeff = (left_coeff + right_coeff) * 0.5
                mode = 'both'
                a = self.lane_width_alpha
                est = (1.0 - a) * self.lane_width_est + a * width
                # El ancho aprendido no puede alejarse más de ±40% del valor inicial
                # (protege contra confundir la línea de otro carril).
                lo, hi = 0.6 * self.lane_width_init, 1.4 * self.lane_width_init
                self.lane_width_est = min(max(est, lo), hi)
            elif self.single_line_fallback:
                # Ancho imposible: una de las dos está mal. Se queda con la que tenga más
                # puntos (la más confiable) y se trata como modo de una línea.
                if n_left >= n_right:
                    right_coeff = None
                else:
                    left_coeff = None
            else:
                self.last_mode = 'none'
                return invalid

        if center_coeff is None and self.single_line_fallback:
            half = 0.5 * self.lane_width_est
            if left_coeff is not None and right_coeff is None and n_left >= self.min_points_single:
                center_coeff = left_coeff.copy()
                center_coeff[-1] += half      # centro = línea izquierda + medio ancho (y a la derecha)
                mode = 'left'
            elif right_coeff is not None and left_coeff is None and n_right >= self.min_points_single:
                center_coeff = right_coeff.copy()
                center_coeff[-1] -= half      # centro = línea derecha - medio ancho
                mode = 'right'

        if center_coeff is None:
            self.last_mode = 'none'
            return invalid

        center_deriv = np.polyder(center_coeff)
        center_second = np.polyder(center_deriv)

        center_y = float(np.polyval(center_coeff, x_eval))
        if abs(center_y) > self.max_abs_lateral_error:
            self.last_mode = 'none'
            return invalid

        center_slope = float(np.polyval(center_deriv, x_eval))
        if abs(center_slope) > self.max_abs_slope:
            self.last_mode = 'none'
            return invalid

        center_curv = float(np.polyval(center_second, x_eval))
        denom = (1.0 + center_slope ** 2) ** 1.5
        curvature_raw = 0.0 if denom <= 1e-6 else center_curv / denom

        if abs(curvature_raw) > self.max_abs_curvature:
            self.last_mode = 'none'
            return invalid

        lateral_error = float(center_y + self.lateral_offset)
        heading_error = float(-np.arctan(center_slope))
        curvature = float(-curvature_raw)

        self.last_mode = mode
        return True, lateral_error, heading_error, curvature, mode
