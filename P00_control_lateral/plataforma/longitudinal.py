"""
Rama longitudinal común, PID y conformador de pedal.

Copias de PIDLongitudinalController y PedalShaper de
mpc_monza_Completo_barrido.py, sin cambios de lógica. El comando vive entre
menos uno y uno, positivo acelerador y negativo freno. Las ganancias del PID
no están sintonizadas y se fijan en la caracterización longitudinal previa a
la campaña, sección 2.5 del manuscrito.
"""
import numpy as np


class PIDLongitudinalController:
    def __init__(self, kp=0.06, ki=0.015, kd=0.01, dt=0.05, limite_integral=50.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.limite_integral = limite_integral
        self.integral = 0.0
        self.prev_error = 0.0

    def compute_control(self, current_speed_kmh, target_speed_kmh):
        error = target_speed_kmh - current_speed_kmh
        # Anti windup por recorte de la integral.
        self.integral = float(np.clip(self.integral + error * self.dt,
                                      -self.limite_integral, self.limite_integral))
        deriv = (error - self.prev_error) / self.dt
        self.prev_error = error
        u = self.kp * error + self.ki * self.integral + self.kd * deriv
        return float(np.clip(u, -1.0, 1.0))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class PedalShaper:
    """Rampa explícita de pedal con constantes de subida y bajada independientes."""

    def __init__(self, ts, throttle_rise_s=0.60, throttle_fall_s=0.35,
                 brake_rise_s=0.45, brake_fall_s=0.30):
        self.d_th_up = ts / max(throttle_rise_s, 1e-3)
        self.d_th_dn = ts / max(throttle_fall_s, 1e-3)
        self.d_br_up = ts / max(brake_rise_s, 1e-3)
        self.d_br_dn = ts / max(brake_fall_s, 1e-3)
        self.u = 0.0

    def shape(self, u_target):
        u_target = float(np.clip(u_target, -1.0, 1.0))
        u = self.u
        if u_target > u:
            step = self.d_th_up if u >= 0.0 else self.d_br_dn
            u = min(u_target, u + step)
        elif u_target < u:
            step = self.d_th_dn if u > 0.0 else self.d_br_up
            u = max(u_target, u - step)
        self.u = u
        return u

    def reset(self):
        self.u = 0.0
