# =============================================================================
# mpc_monza_UNIFICADO.py
# UN SOLO LOOP: MPC de dirección + MPC longitudinal (modelo K/tau validado)
# corriendo juntos, mandando los 3 comandos (steer, gas, brake) a vJoy cada
# ciclo.
#
# CÓMO CORRERLO:
#   python mpc_monza_UNIFICADO.py              → Fase A, volante simulado
#   python mpc_monza_UNIFICADO.py --ffbeast     → Fase B, volante físico FFBeast
#
# REQUISITOS PREVIOS (ya deberías tenerlos listos si seguiste los pasos
# anteriores):
#   - monza_fast_lane.csv en la misma carpeta
#   - AC abierto con sesión activa en pista
#   - En AC: Steering=vJoy eje X, Throttle=vJoy eje Y, Brake=vJoy eje RZ
#   - Gatillos físicos DESVINCULADOS de Throttle/Brake en AC (para que no
#     compitan con lo que manda el MPC)
# =============================================================================

import ctypes
import mmap
import math
import time
import csv
import numpy as np
from scipy.optimize import minimize
import pyvjoy

try:
    import sdl2
    import sdl2.ext
    SDL2_AVAILABLE = True
except ImportError:
    SDL2_AVAILABLE = False


# =============================================================================
# SECCIÓN 1 — SHARED MEMORY DE AC (sin cambios)
# =============================================================================

class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int32),
        ("gas", ctypes.c_float),
        ("brake", ctypes.c_float),
        ("fuel", ctypes.c_float),
        ("gear", ctypes.c_int32),
        ("rpms", ctypes.c_int32),
        ("steerAngle", ctypes.c_float),
        ("speedKmh", ctypes.c_float),
        ("velocity", ctypes.c_float * 3),
        ("accG", ctypes.c_float * 3),
        ("wheelSlip", ctypes.c_float * 4),
        ("wheelLoad", ctypes.c_float * 4),
        ("wheelsPressure", ctypes.c_float * 4),
        ("wheelAngularSpeed", ctypes.c_float * 4),
        ("tyreWear", ctypes.c_float * 4),
        ("tyreDirtyLevel", ctypes.c_float * 4),
        ("tyreCoreTemperature", ctypes.c_float * 4),
        ("camberRAD", ctypes.c_float * 4),
        ("suspensionTravel", ctypes.c_float * 4),
        ("drs", ctypes.c_float),
        ("tc", ctypes.c_float),
        ("heading", ctypes.c_float),
        ("pitch", ctypes.c_float),
        ("roll", ctypes.c_float),
        ("cgHeight", ctypes.c_float),
        ("carDamage", ctypes.c_float * 5),
        ("numberOfTyresOut", ctypes.c_int32),
        ("pitLimiterOn", ctypes.c_int32),
        ("abs", ctypes.c_float),
    ]

class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", ctypes.c_int32),
        ("status", ctypes.c_int32),
        ("session", ctypes.c_int32),
        ("currentTime", ctypes.c_wchar * 15),
        ("lastTime", ctypes.c_wchar * 15),
        ("bestTime", ctypes.c_wchar * 15),
        ("split", ctypes.c_wchar * 15),
        ("completedLaps", ctypes.c_int32),
        ("position", ctypes.c_int32),
        ("iCurrentTime", ctypes.c_int32),
        ("iLastTime", ctypes.c_int32),
        ("iBestTime", ctypes.c_int32),
        ("sessionTimeLeft", ctypes.c_float),
        ("distanceTraveled", ctypes.c_float),
        ("isInPit", ctypes.c_int32),
        ("currentSectorIndex", ctypes.c_int32),
        ("lastSectorTime", ctypes.c_int32),
        ("numberOfLaps", ctypes.c_int32),
        ("tyreCompound", ctypes.c_wchar * 33),
        ("replayTimeMultiplier", ctypes.c_float),
        ("normalizedCarPosition", ctypes.c_float),
        ("carCoordinates", ctypes.c_float * 3),
    ]

class ACSharedMemory:
    def __init__(self):
        self._ph_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysics), "acpmf_physics")
        self._gr_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFileGraphic), "acpmf_graphics")

    def read_physics(self) -> SPageFilePhysics:
        return SPageFilePhysics.from_buffer_copy(self._ph_mmap)

    def read_graphics(self) -> SPageFileGraphic:
        return SPageFileGraphic.from_buffer_copy(self._gr_mmap)

    def close(self):
        self._ph_mmap.close()
        self._gr_mmap.close()


# =============================================================================
# SECCIÓN 2 — TRAZADA Y FRENET (sin cambios)
# =============================================================================

class ReferencePath:
    def __init__(self, csv_path: str):
        xs, zs, s = [], [], []
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["x"]))
                zs.append(float(row["z"]))
                s.append(float(row["cumulative_length_m"]))

        self.x = np.array(xs)
        self.z = np.array(zs)
        self.s = np.array(s)
        self.n = len(self.x)
        self.total_length = self.s[-1]
        self.avg_spacing = self.total_length / self.n

        dx = np.roll(self.x, -1) - self.x
        dz = np.roll(self.z, -1) - self.z
        self.heading = np.arctan2(dz, dx)

        dtheta = np.roll(self.heading, -1) - self.heading
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        next_x = np.roll(self.x, -1)
        next_z = np.roll(self.z, -1)
        ds = np.sqrt((next_x - self.x) ** 2 + (next_z - self.z) ** 2)
        ds[ds < 1e-3] = 1e-3
        self.curvature = dtheta / ds

        self._last_idx = 0

    def nearest(self, car_x, car_z, window=60):
        idxs = (self._last_idx + np.arange(-window, window)) % self.n
        d2 = (self.x[idxs] - car_x) ** 2 + (self.z[idxs] - car_z) ** 2
        best = idxs[np.argmin(d2)]
        if d2.min() > 400:
            d2g = (self.x - car_x) ** 2 + (self.z - car_z) ** 2
            best = int(np.argmin(d2g))
        self._last_idx = int(best)
        return self._last_idx

    def frenet(self, car_x, car_z, car_heading):
        idx = self.nearest(car_x, car_z)
        ph = self.heading[idx]
        dx = car_x - self.x[idx]
        dz = car_z - self.z[idx]
        e_y = -math.sin(ph) * dx + math.cos(ph) * dz
        e_psi = (car_heading - ph + math.pi) % (2 * math.pi) - math.pi
        return e_y, e_psi, self.s[idx], idx

    def curvature_at_offset(self, idx, dist_m):
        idx_step = max(1, round(dist_m / self.avg_spacing))
        return float(self.curvature[(idx + idx_step) % self.n])


# =============================================================================
# SECCIÓN 3 — REFERENCIA DE DIRECCIÓN (sin cambios)
# =============================================================================

class ReferenceGenerator:
    def __init__(self, wheelbase_L, steering_ratio_n, Ld_base=3.0, error_decay=0.998,
                 steer_sign=-1):
        self.L = wheelbase_L
        self.n = steering_ratio_n
        self.Ld_base = Ld_base
        self.error_decay = error_decay
        self.steer_sign = steer_sign

    def generate(self, e_y, e_psi, path: ReferencePath, idx, speed_ms, N, Ts):
        Ld = max(5.0, self.Ld_base + 0.5 * speed_ms)
        alpha0 = e_psi + math.atan2(0.3 * e_y, Ld)
        delta_pp0 = math.atan2(2 * self.L * math.sin(alpha0), Ld)
        dist_per_step = max(0.5, speed_ms * Ts)

        theta_ref = np.zeros(N)
        for k in range(N):
            kappa_k = path.curvature_at_offset(idx, dist_per_step * (k + 1))
            delta_ff = self.L * kappa_k
            decay = self.error_decay ** k
            delta_total = self.steer_sign * (decay * delta_pp0 + 0.5 * delta_ff)
            theta_ref[k] = np.clip(delta_total * self.n, -1.2, 1.2)
        return theta_ref


# =============================================================================
# SECCIÓN 3B — PERFIL DE VELOCIDAD (sin cambios; salida en m/s)
# =============================================================================

class SpeedProfileGenerator:
    def __init__(self, path: ReferencePath,
                 ay_max_ms2=6.5, ax_brake_max_ms2=5.5,
                 grip_usage_factor=0.75, v_max_recta_ms=83.3,
                 n_backward_passes=3):
        self.path = path
        self.ay_max = ay_max_ms2 * grip_usage_factor
        self.ax_brake_max = ax_brake_max_ms2 * grip_usage_factor
        self.v_cap = v_max_recta_ms

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

        self.v_max = v_max  # m/s

    def v_ref_at_offset(self, idx, dist_m):
        idx_step = max(1, round(dist_m / self.path.avg_spacing))
        return float(self.v_max[(idx + idx_step) % self.path.n])

    def target_speed_kmh(self, idx, speed_ms, preview_m=15.0):
        """
        Devuelve UN solo valor objetivo en km/h (no una secuencia): el
        MPCLongitudinalController que ya validaste toma target_speed
        escalar, no una secuencia por paso — a diferencia del MPC de
        dirección, aquí es el propio horizonte interno del controlador el
        que ya mira hacia adelante con el modelo K/tau; lo que este método
        aporta es QUÉ velocidad objetivo usar en cada instante, adelantada
        `preview_m` metros para que el MPC ya esté frenando antes de la
        curva, no en cuanto la detecta.
        """
        v_ref_ms = self.v_ref_at_offset(idx, preview_m)
        return v_ref_ms * 3.6


# =============================================================================
# SECCIÓN 4 — MPC DE DIRECCIÓN (sin cambios)
# =============================================================================

class MPCSteeringController:
    def __init__(self, J=0.02, b=0.1, Ts=0.05, N=10,
                 Q=10.41, Qf=10.41, R=0.0, Rd=0.288,
                 tau_max=2.0, rate_max=1.0, theta_max=1.2):
        self.J, self.b, self.Ts, self.N = J, b, Ts, N
        self.Q, self.Qf, self.R, self.Rd = Q, Qf, R, Rd
        self.tau_max, self.rate_max, self.theta_max = tau_max, rate_max, theta_max
        self.A = np.array([[1.0, Ts], [0.0, 1.0 - Ts * b / J]])
        self.B = np.array([0.0, Ts / J])
        self.u_prev = 0.0
        self._u_warm = np.zeros(N)

    def _predict(self, x0, u_seq):
        xs = np.zeros((self.N + 1, 2))
        xs[0] = x0
        for k in range(self.N):
            xs[k + 1] = self.A @ xs[k] + self.B * u_seq[k]
        return xs

    def _cost(self, u_seq, x0, theta_ref_seq, u_prev):
        xs = self._predict(x0, u_seq)
        cost = 0.0
        for k in range(1, self.N + 1):
            w = self.Qf if k == self.N else self.Q
            cost += w * (xs[k, 0] - theta_ref_seq[k - 1]) ** 2
        prev = u_prev
        for u in u_seq:
            cost += self.R * u ** 2
            cost += self.Rd * (u - prev) ** 2
            prev = u
        return cost

    def solve(self, theta_current, theta_dot_current, theta_ref_seq):
        x0 = np.array([theta_current, theta_dot_current])
        bounds = [(-self.tau_max, self.tau_max)] * self.N

        def rate_constraint(u_seq):
            du = np.diff(np.concatenate(([self.u_prev], u_seq)))
            return self.rate_max - np.abs(du)

        def theta_bound_constraint(u_seq):
            xs = self._predict(x0, u_seq)
            return self.theta_max - np.abs(xs[1:, 0])

        constraints = [
            {"type": "ineq", "fun": rate_constraint},
            {"type": "ineq", "fun": theta_bound_constraint},
        ]
        res = minimize(self._cost, self._u_warm, args=(x0, theta_ref_seq, self.u_prev),
                        method="SLSQP", bounds=bounds, constraints=constraints,
                        options={"maxiter": 40, "ftol": 1e-6})
        u_opt = res.x if res.success else self._u_warm
        tau_apply = float(np.clip(u_opt[0], -self.tau_max, self.tau_max))
        self._u_warm = np.concatenate([u_opt[1:], [u_opt[-1]]])
        self.u_prev = tau_apply
        x1 = self.A @ x0 + self.B * tau_apply
        return tau_apply, x1[0], x1[1]


# =============================================================================
# SECCIÓN 4B — MPC LONGITUDINAL — TU MODELO K/TAU VALIDADO
# =============================================================================

class MPCLongitudinalController:
    """
    NOTA sobre Ts vs Ts_pred: `ts` (el paso de tiempo usado en la
    predicción interna del modelo) puede ser MÁS GRANDE que la frecuencia
    real a la que este controlador se llama y aplica comandos.

    Esto es intencional: tu dinámica de aceleración tiene τ≈2.8s (mucho más
    lenta que la de dirección, τ≈0.2s). Si predices con el mismo Ts=0.05s
    que usas para actuar, tu horizonte de N=15 pasos solo cubre 0.75s —
    menos de un tercio de τ, así que el MPC nunca "ve" su propia dinámica
    asentarse, es miope respecto a su propia física.

    Al predecir con ts=0.2s mientras sigues APLICANDO el u_0* resultante
    cada 0.05s (recompute cada ciclo, horizonte deslizante como siempre),
    el mismo N=15 ahora cubre 3s de horizonte — sí alcanza a cubrir τ.
    El loop de actuación no cambia de frecuencia, solo la resolución
    temporal con la que el modelo interno predice el futuro.

    MODELO — DOS ESTRUCTURAS DISTINTAS, NO UNA (cambio importante respecto
    a versiones anteriores de este archivo):

      THROTTLE (u >= 0): primer orden, igual que antes.
          v[k+1] = v[k] + (ts/tau_th) * ( -v[k] + K_th*u )
        Tiene sentido como saturación SUAVE (resistencia aerodinámica
        creciente con v). Identificado por step-response real.

      BRAKE (u < 0): desaceleración CONSTANTE, no exponencial.
          v[k+1] = max(0, v[k] - ts * a_brake_max * |u| )
        Este cambio se hizo después de ver los datos reales de frenado:
        la velocidad decae casi perfectamente en LÍNEA RECTA (R²≈0.999
        con un ajuste lineal, contra R²≈0.83-0.94 forzando una
        exponencial). Tiene sentido físico: el freno aplica una fuerza de
        fricción casi constante, no una fuerza que decae con v como el
        arrastre aerodinámico — es la misma física que ya asume
        ax_brake_max en SpeedProfileGenerator, así que este cambio
        también hace que el MPC longitudinal y el generador de perfil de
        velocidad queden usando el MISMO modelo de frenado.

        Un hallazgo de la calibración real: a_brake salió CASI IDÉNTICO
        (10.1-10.5 m/s²) en las 4 amplitudes de prueba (0.3 a 1.0) — el
        auto ya está cerca del límite de agarre incluso con freno parcial,
        así que este modelo usa a_brake_max escalado por |u| de forma
        conservadora (asume que a u=1.0 sí obtienes el a_brake_max medido,
        y escala proporcionalmente hacia abajo para u menor) — subestima
        un poco el frenado a pedal parcial en vez de sobreestimarlo, que
        es la dirección segura del error.
    """
    def __init__(self, model_params, ts=0.2, horizon=15):
        self.ts = ts
        self.N = horizon
        self.K_th = model_params['throttle']['K']
        self.tau_th = max(model_params['throttle']['tau'], 1e-3)
        self.a_brake_max = model_params['brake_a_max_ms2'] * 3.6  # a m/s² -> km/h/s, mismas unidades que v (km/h)
        self.Q = 10.0
        self.R = 0.5
        self.R_rate = 2.0
        self.u_prev = 0.0
        self._u_warm = np.zeros(horizon)

    def predict_velocity(self, v0, u_sequence):
        v_pred = np.zeros(self.N + 1)
        v_pred[0] = v0
        v = v0
        for k in range(self.N):
            u = u_sequence[k]
            if u >= 0:
                v += (self.ts / self.tau_th) * (-v + self.K_th * u)
            else:
                v -= self.ts * self.a_brake_max * (-u)
                v = max(0.0, v)
            v_pred[k + 1] = v
        return v_pred

    def cost_function(self, u_sequence, v0, v_ref):
        v_pred = self.predict_velocity(v0, u_sequence)
        cost_tracking = np.sum(self.Q * (v_pred[1:] - v_ref) ** 2)
        cost_effort = np.sum(self.R * u_sequence ** 2)
        u_extended = np.concatenate([[self.u_prev], u_sequence])
        cost_rate = np.sum(self.R_rate * np.diff(u_extended) ** 2)
        return cost_tracking + cost_effort + cost_rate

    def compute_control(self, current_speed_kmh, target_speed_kmh):
        v_ref = np.full(self.N, target_speed_kmh)
        bounds = [(-1.0, 1.0)] * self.N
        res = minimize(self.cost_function, self._u_warm, args=(current_speed_kmh, v_ref),
                        method='L-BFGS-B', bounds=bounds,
                        options={'maxiter': 30, 'ftol': 1e-2})
        u_seq = res.x if res.success else np.full(self.N, -0.3)
        u_optimal = float(np.clip(u_seq[0], -1.0, 1.0))
        self._u_warm = np.concatenate([u_seq[1:], [u_seq[-1]]])
        self.u_prev = u_optimal
        return u_optimal


# =============================================================================
# SECCIÓN 4C — KALMAN DE DIRECCIÓN (sin cambios, solo Fase B)
# =============================================================================

class SteeringKalmanFilter:
    def __init__(self, A, B, q_theta=1e-6, q_theta_dot=1e-2, r_theta=1e-4,
                 theta0=0.0, theta_dot0=0.0):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float).reshape(2)
        self.H = np.array([[1.0, 0.0]])
        self.Q = np.diag([q_theta, q_theta_dot])
        self.R = np.array([[r_theta]])
        self.x = np.array([theta0, theta_dot0], dtype=float)
        self.P = np.eye(2) * 1e-3

    def predict(self, u):
        self.x = self.A @ self.x + self.B * u
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, theta_meas):
        z = np.array([theta_meas])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = (self.P @ self.H.T @ np.linalg.inv(S)).flatten()
        self.x = self.x + K * y[0]
        self.P = (np.eye(2) - np.outer(K, self.H.flatten())) @ self.P

    def state(self):
        return float(self.x[0]), float(self.x[1])

    def reset(self, theta0=0.0, theta_dot0=0.0):
        self.x = np.array([theta0, theta_dot0], dtype=float)
        self.P = np.eye(2) * 1e-3


# =============================================================================
# SECCIÓN 5 — PLANTAS DEL VOLANTE (sin cambios)
# =============================================================================

class SimulatedSteeringPlant:
    def __init__(self, J=0.02, b=0.1, Ts=0.05):
        self.J, self.b, self.Ts = J, b, Ts
        self.theta = 0.0
        self.theta_dot = 0.0

    def get_state(self):
        return self.theta, self.theta_dot

    def apply_torque(self, tau):
        theta_ddot = (tau - self.b * self.theta_dot) / self.J
        self.theta_dot += theta_ddot * self.Ts
        self.theta += self.theta_dot * self.Ts
        self.theta = float(np.clip(self.theta, -1.2, 1.2))
        return self.theta, self.theta_dot

    def center(self):
        self.theta = 0.0
        self.theta_dot = 0.0

    def close(self):
        pass


class FFBeastSteeringPlant:
    def __init__(self, max_steering_rad=math.radians(450), tau_max_motor=2.0,
                 direction_sign=1, joystick_index=None, axis_index=0):
        if not SDL2_AVAILABLE:
            raise RuntimeError("Instala pysdl2: pip install pysdl2 pysdl2-dll")
        self.max_steering_rad = max_steering_rad
        self.tau_max_motor = tau_max_motor
        self.nm_to_magnitude = 32767.0 / tau_max_motor
        self.direction_sign = direction_sign
        self.axis_index = axis_index

        if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC) != 0:
            raise RuntimeError(f"SDL_Init falló: {sdl2.SDL_GetError()}")
        n = sdl2.SDL_NumJoysticks()
        if n == 0:
            raise RuntimeError("No se detectó ningún joystick/volante conectado.")
        if joystick_index is None:
            joystick_index = 0
        self.joystick = sdl2.SDL_JoystickOpen(joystick_index)
        if not self.joystick:
            raise RuntimeError(f"No se pudo abrir el joystick: {sdl2.SDL_GetError()}")
        self.haptic = sdl2.SDL_HapticOpenFromJoystick(self.joystick)
        if not self.haptic:
            raise RuntimeError(f"SDL_HapticOpenFromJoystick falló: {sdl2.SDL_GetError()}")

        effect = sdl2.SDL_HapticEffect()
        ctypes.memset(ctypes.byref(effect), 0, ctypes.sizeof(effect))
        effect.type = sdl2.SDL_HAPTIC_CONSTANT
        effect.constant.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
        effect.constant.direction.dir[0] = 1
        effect.constant.length = sdl2.SDL_HAPTIC_INFINITY
        effect.constant.level = 0
        self._effect = effect
        self.effect_id = sdl2.SDL_HapticNewEffect(self.haptic, ctypes.byref(self._effect))
        sdl2.SDL_HapticRunEffect(self.haptic, self.effect_id, 1)

        self._last_theta = 0.0
        self._last_t = time.time()

    def _read_axis_rad(self):
        raw = sdl2.SDL_JoystickGetAxis(self.joystick, self.axis_index)
        return (raw / 32768.0) * self.max_steering_rad

    def get_state(self):
        theta = self._read_axis_rad()
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        theta_dot = (theta - self._last_theta) / dt
        self._last_theta = theta
        self._last_t = now
        return theta, theta_dot

    def apply_torque(self, tau):
        tau_clipped = float(np.clip(tau, -self.tau_max_motor, self.tau_max_motor))
        magnitude = int(np.clip(self.direction_sign * tau_clipped * self.nm_to_magnitude, -32767, 32767))
        self._effect.constant.level = magnitude
        sdl2.SDL_HapticUpdateEffect(self.haptic, self.effect_id, ctypes.byref(self._effect))
        return self.get_state()

    def center(self):
        try:
            self._effect.constant.level = 0
            sdl2.SDL_HapticUpdateEffect(self.haptic, self.effect_id, ctypes.byref(self._effect))
        except Exception:
            pass

    def close(self):
        try:
            sdl2.SDL_HapticStopEffect(self.haptic, self.effect_id)
            sdl2.SDL_HapticDestroyEffect(self.haptic, self.effect_id)
            sdl2.SDL_HapticClose(self.haptic)
            sdl2.SDL_JoystickClose(self.joystick)
        except Exception:
            pass


# =============================================================================
# SECCIÓN 6 — SALIDA A VJOY (3 ejes: X=steer, Y=gas, RZ=freno)
# =============================================================================

class VJoyOutput:
    def __init__(self, device_id=1):
        self.j = pyvjoy.VJoyDevice(device_id)
        self.center()

    def send_all(self, steer_rad, u_cmd, max_rad=1.2):
        """u_cmd viene directo del MPC longitudinal: [-1, 1], + gas, - freno."""
        norm = np.clip(steer_rad / max_rad, -1.0, 1.0)
        steer_val = max(1, min(32767, int((norm + 1.0) / 2.0 * 32766 + 1)))

        if u_cmd >= 0:
            gas_val, brake_val = int(u_cmd * 32767), 0
        else:
            gas_val, brake_val = 0, int(-u_cmd * 32767)

        self.j.data.wAxisX = steer_val
        self.j.data.wAxisY = gas_val
        self.j.data.wAxisZRot = brake_val
        self.j.update()
        return gas_val / 32767.0, brake_val / 32767.0

    def center(self):
        self.j.data.wAxisX = 16384
        self.j.data.wAxisY = 0
        self.j.data.wAxisZRot = 0
        self.j.update()


# =============================================================================
# SECCIÓN 7 — LOOP PRINCIPAL ÚNICO
# =============================================================================

def main(use_ffbeast=False):
    print("=" * 60)
    print("  MPC UNIFICADO — Dirección + Gas/Freno | Monza")
    print("=" * 60)

    print("\n[1/6] Cargando trazada...")
    path = ReferencePath("monza_fast_lane.csv")
    print(f"      {path.n} puntos | {path.total_length:.0f} m ✓")

    print("[2/6] Calculando perfil de velocidad (conservador)...")
    speed_gen = SpeedProfileGenerator(path, ay_max_ms2=6.5, ax_brake_max_ms2=5.5, grip_usage_factor=0.75)
    print(f"      v_max: {speed_gen.v_max.min()*3.6:.0f}–{speed_gen.v_max.max()*3.6:.0f} km/h ✓")

    print("[3/6] Conectando a AC (shared memory)...")
    try:
        sm = ACSharedMemory()
        print("      ✓")
    except Exception as e:
        print(f"      ERROR: {e}")
        return

    print("[4/6] Conectando vJoy (X=steer, Y=gas, RZ=brake)...")
    try:
        vjoy = VJoyOutput(device_id=1)
        print("      ✓  (verifica que en AC ya asignaste estos 3 ejes)")
    except Exception as e:
        print(f"      ERROR: {e}")
        return

    print("[5/6] Inicializando planta de dirección...")
    if use_ffbeast:
        try:
            plant = FFBeastSteeringPlant()
            print("      FFBeast conectado ✓")
        except Exception as e:
            print(f"      ERROR: {e}")
            return
    else:
        plant = SimulatedSteeringPlant(Ts=0.05)
        print("      Planta simulada ✓")

    print("[6/6] Inicializando ambos MPC...")
    Ts = 0.05
    N_steer = 10
    ref_gen = ReferenceGenerator(wheelbase_L=2.7, steering_ratio_n=16, steer_sign=-1)
    mpc_steer = MPCSteeringController(Ts=Ts, N=N_steer)

    model_params = {
        # Throttle: modelo K/tau identificado con step_test_logger.py +
        # identify_model.py sobre 4 pruebas (u=0.3/0.5/0.7/1.0), gear=2,
        # tau=2.81s (consistente entre amplitudes, std=0.06s — confiable).
        #
        # K = 122.8 es el PROMEDIO EMPÍRICO de las 4 pruebas, pero está
        # contaminado por el limitador de RPM de la marcha usada (el auto
        # se topó con un techo de ~53.7 km/h igual en las 4 pruebas, así
        # que K*u salió casi constante entre amplitudes — no es un K
        # "limpio" de ganancia real). SUSTITUYE este valor por uno basado
        # en la velocidad máxima real de tu auto en Monza (ej. si llega a
        # ~250 km/h a fondo, usa K≈250) — ver README_acelerador_freno.md,
        # sección 5, para el detalle completo de por qué.
        'throttle': {'K': 122.8, 'tau': 2.81},

        # Brake: YA NO es K/tau — ver el docstring de MPCLongitudinalController
        # más arriba para la explicación completa del cambio de modelo.
        # a_brake_max identificado con el mismo flujo: desaceleración casi
        # idéntica (10.1-10.5 m/s², R²≈0.999) en las 4 amplitudes de prueba.
        'brake_a_max_ms2': 10.25,
    }
    mpc_speed = MPCLongitudinalController(model_params, ts=0.2, horizon=15)
    print(f"      Dirección: N={N_steer} pasos × {Ts}s = {N_steer*Ts:.2f}s horizonte (τ_dir≈0.2s)")
    print(f"      Velocidad: N=15 pasos × 0.2s = 3.0s horizonte (τ_throttle≈2.81s) ✓")

    kf = None
    if use_ffbeast:
        kf = SteeringKalmanFilter(mpc_steer.A, mpc_steer.B,
                                   q_theta=1e-6, q_theta_dot=1e-2, r_theta=1e-4)

    log_name = f"mpc_unificado_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_name, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['t_s', 'e_y_m', 'theta_ref0_deg', 'theta_real_deg', 'tau_Nm',
                      'v_real_kmh', 'v_target_kmh', 'u_cmd', 'gas', 'brake', 'loop_dt_ms'])

    print()
    if use_ffbeast:
        print("ADVERTENCIA: el FFBeast va a mover el motor físico del volante.")
        print("             Corte de emergencia a la mano, no solo Ctrl+C.")
    print("ADVERTENCIA: el sistema toma control TOTAL (dirección + gas + freno).")
    print()
    input(">>> Carro detenido en pista, Enter para ACTIVAR <<<")
    print("\n¡MPC ACTIVO!\n")

    last_packet_id = None
    last_heading = 0.0
    last_tau_applied = 0.0
    MIN_SPEED_MS = 0.5
    cycle = 0
    t0 = time.time()

    print(f"{'t(s)':>6} | {'e_y(m)':>7} | {'θ(°)':>7} | {'v(km/h)':>8} | "
          f"{'v_obj(km/h)':>11} | {'gas':>5} | {'brake':>5}")
    print("-" * 75)

    try:
        while True:
            t_loop = time.time()
            gr = sm.read_graphics()
            ph = sm.read_physics()

            if gr.packetId == last_packet_id:
                time.sleep(0.005)
                continue
            last_packet_id = gr.packetId

            car_x, car_y, car_z = gr.carCoordinates
            speed_ms = ph.speedKmh / 3.6
            speed_kmh = ph.speedKmh

            car_vx, _, car_vz = ph.velocity
            if speed_ms > MIN_SPEED_MS:
                car_heading = math.atan2(car_vz, car_vx)
                last_heading = car_heading
            else:
                car_heading = last_heading

            e_y, e_psi, s, idx = path.frenet(car_x, car_z, car_heading)

            # --- Dirección ---
            if use_ffbeast:
                theta_meas, _ = plant.get_state()
                kf.predict(last_tau_applied)
                kf.update(theta_meas)
                theta_current, theta_dot_current = kf.state()
            else:
                theta_current, theta_dot_current = plant.get_state()

            theta_ref_seq = ref_gen.generate(e_y, e_psi, path, idx, speed_ms, N_steer, Ts)
            t_steer0 = time.time()
            tau, _, _ = mpc_steer.solve(theta_current, theta_dot_current, theta_ref_seq)
            t_steer_ms = (time.time() - t_steer0) * 1000.0
            theta_new, _ = plant.apply_torque(tau)
            last_tau_applied = tau

            # --- Velocidad ---
            v_target_kmh = speed_gen.target_speed_kmh(idx, speed_ms, preview_m=15.0)
            t_speed0 = time.time()
            u_cmd = mpc_speed.compute_control(speed_kmh, v_target_kmh)
            t_speed_ms = (time.time() - t_speed0) * 1000.0

            # --- Salida conjunta ---
            gas, brake = vjoy.send_all(theta_new, u_cmd)

            cycle += 1
            t_elapsed = time.time() - t0
            loop_dt_ms = (time.time() - t_loop) * 1000.0

            if cycle % 10 == 0:
                print(f"{t_elapsed:>6.1f} | {e_y:>7.2f} | {math.degrees(theta_new):>7.1f} | "
                      f"{speed_kmh:>8.1f} | {v_target_kmh:>11.1f} | {gas:>5.2f} | {brake:>5.2f} | "
                      f"solve: dir={t_steer_ms:.1f}ms vel={t_speed_ms:.1f}ms tot_loop={loop_dt_ms:.1f}ms")

            writer.writerow([round(t_elapsed, 3), round(e_y, 4),
                              round(math.degrees(theta_ref_seq[0]), 3),
                              round(math.degrees(theta_new), 3), round(tau, 4),
                              round(speed_kmh, 2), round(v_target_kmh, 2),
                              round(u_cmd, 3), round(gas, 3), round(brake, 3),
                              round(loop_dt_ms, 2)])
            if cycle % 20 == 0:
                log_file.flush()

            elapsed = time.time() - t_loop
            sleep_t = Ts - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n\nMPC detenido por el usuario")
    finally:
        plant.center()
        vjoy.center()
        plant.close()
        log_file.flush()
        log_file.close()
        sm.close()
        print(f"Log guardado: {log_name}")
        print(f"Ciclos ejecutados: {cycle}")


if __name__ == "__main__":
    import sys
    use_ffbeast = "--ffbeast" in sys.argv
    main(use_ffbeast=use_ffbeast)
