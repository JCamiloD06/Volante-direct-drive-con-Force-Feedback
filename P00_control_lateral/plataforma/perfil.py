"""
Perfil de velocidad común.

SpeedProfileGenerator es copia de mpc_monza_Completo_barrido.py, límite por
curvatura, barrido hacia atrás con deceleración de lazo cerrado y suavizado
que solo puede bajar el perfil. PerfilEscalado es nuevo y aplica la decisión
3.1, perfil base completo multiplicado por un factor f.
"""
import math

import numpy as np


class SpeedProfileGenerator:
    def __init__(self, path, ay_max_ms2=6.5, ax_brake_max_ms2=10.25,
                 grip_usage_factor=0.5, grip_usage_brake=None,
                 v_max_recta_ms=50, n_backward_passes=5, profile_smooth_m=25.0):
        self.path = path
        if grip_usage_brake is None:
            grip_usage_brake = grip_usage_factor
        self.ay_max = ay_max_ms2 * grip_usage_factor
        self.ax_brake_max = ax_brake_max_ms2 * grip_usage_brake
        self.v_cap = v_max_recta_ms
        self.profile_smooth_m = profile_smooth_m

        kappa = np.abs(path.curvature)
        kappa_safe = np.maximum(kappa, 1e-5)
        v_curve = np.sqrt(self.ay_max / kappa_safe)
        v_curve = np.minimum(v_curve, self.v_cap)

        next_x = np.roll(path.x, -1)
        next_z = np.roll(path.z, -1)
        ds = np.sqrt((next_x - path.x) ** 2 + (next_z - path.z) ** 2)
        ds[ds < 1e-3] = path.avg_spacing

        v_max = v_curve.copy()
        for _ in range(n_backward_passes):
            for i in range(path.n - 1, -1, -1):
                nxt = (i + 1) % path.n
                v_allowed = math.sqrt(max(0.0, v_max[nxt] ** 2 + 2 * self.ax_brake_max * ds[i]))
                if v_allowed < v_max[i]:
                    v_max[i] = v_allowed

        self.v_limit = v_max.copy()

        win = max(1, int(round(self.profile_smooth_m / path.avg_spacing)))
        if win % 2 == 0:
            win += 1
        if win >= 3:
            kernel = np.ones(win) / win
            ext = np.concatenate([v_max[-(win // 2):], v_max, v_max[:win // 2]])
            v_max = np.minimum(np.convolve(ext, kernel, mode="valid"), v_max)

        self.v_max = v_max

    def v_ref_min_ahead(self, idx, preview_m):
        steps = max(1, int(round(preview_m / self.path.avg_spacing)))
        window_idx = (idx + np.arange(0, steps + 1)) % self.path.n
        return float(np.min(self.v_max[window_idx]))

    def target_speed_kmh(self, idx, speed_ms, preview_m=20.0, preview_speed_gain=0.35):
        preview_eff = preview_m + preview_speed_gain * max(0.0, speed_ms)
        return self.v_ref_min_ahead(idx, preview_eff) * 3.6


class PerfilEscalado:
    """Perfil base multiplicado por f. Decisión 3.1."""

    def __init__(self, base, factor, preview_m, preview_speed_gain):
        self.base = base
        self.path = base.path
        self.factor = float(factor)
        self.v_max = base.v_max * self.factor
        self.v_limit = base.v_limit * self.factor
        self.preview_m = preview_m
        self.preview_speed_gain = preview_speed_gain

    def velocidad_perfil_kmh(self, idx):
        return float(self.v_max[idx]) * 3.6

    def objetivo_kmh(self, idx, speed_ms):
        preview_eff = self.preview_m + self.preview_speed_gain * max(0.0, speed_ms)
        steps = max(1, int(round(preview_eff / self.path.avg_spacing)))
        window_idx = (idx + np.arange(0, steps + 1)) % self.path.n
        return float(np.min(self.v_max[window_idx])) * 3.6


def construir_perfil(trazada, cfg_perfil, nombre_perfil):
    base = SpeedProfileGenerator(
        trazada,
        ay_max_ms2=cfg_perfil["ay_max_ms2"],
        ax_brake_max_ms2=cfg_perfil["ax_frenado_max_ms2"],
        grip_usage_factor=cfg_perfil["grip_usage_factor"],
        grip_usage_brake=cfg_perfil["grip_usage_brake"],
        v_max_recta_ms=cfg_perfil["v_max_recta_ms"],
        profile_smooth_m=cfg_perfil["suavizado_perfil_m"],
    )
    factor = cfg_perfil["factores"][nombre_perfil]
    return PerfilEscalado(base, factor, cfg_perfil["preview_m"], cfg_perfil["preview_speed_gain"])
