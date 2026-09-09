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
import os
import sys
import json
import hashlib
import platform
import numpy as np
import scipy
from scipy.optimize import minimize
import pyvjoy

scipy_version = scipy.__version__

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

class SPageFileStaticHead(ctypes.Structure):
    """
    Solo el ENCABEZADO de la página estática, hasta el nombre de la pista.

    Se lee únicamente para dejar en el manifiesto de la corrida qué coche y
    qué pista se usaron. El resto de la página no se declara a propósito,
    porque no hace falta y declararla de memoria sería inventar el
    esquema. Aun así el contenido se valida antes de usarlo, y si no
    resulta legible se registra como no disponible.

    Corrección verificada el 2026-09-09 contra una corrida real. La versión
    anterior declaraba un campo packetId al principio y devolvía las tres
    cadenas desplazadas exactamente cuatro bytes, es decir dos caracteres
    menos en cada una. Se leía fa_romeo_giulietta_qv en lugar de
    alfa_romeo_giulietta_qv, nza en lugar de monza y 16.4 en lugar de
    1.16.4. El desfase aparecía ya en acVersion, lo que sitúa el error
    antes de ese campo. Esta página no lleva packetId.
    """
    _pack_ = 4
    _fields_ = [
        ("smVersion", ctypes.c_wchar * 15),
        ("acVersion", ctypes.c_wchar * 15),
        ("numberOfSessions", ctypes.c_int32),
        ("numCars", ctypes.c_int32),
        ("carModel", ctypes.c_wchar * 33),
        ("track", ctypes.c_wchar * 33),
    ]


class ACSharedMemory:
    def __init__(self):
        self._ph_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysics), "acpmf_physics")
        self._gr_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFileGraphic), "acpmf_graphics")
        try:
            self._st_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFileStaticHead), "acpmf_static")
        except Exception:
            self._st_mmap = None

    def read_physics(self) -> SPageFilePhysics:
        return SPageFilePhysics.from_buffer_copy(self._ph_mmap)

    def read_graphics(self) -> SPageFileGraphic:
        return SPageFileGraphic.from_buffer_copy(self._gr_mmap)

    def read_static_info(self) -> dict:
        """Devuelve coche, pista y versión, o None en cada campo ilegible."""
        info = {"carModel": None, "track": None, "acVersion": None,
                "lectura": "no disponible"}
        if self._st_mmap is None:
            return info

        def limpio(valor):
            texto = str(valor).split("\x00")[0].strip()
            if not texto or not all(32 <= ord(c) < 127 for c in texto):
                return None
            return texto

        try:
            st = SPageFileStaticHead.from_buffer_copy(self._st_mmap)
            info["carModel"] = limpio(st.carModel)
            info["track"] = limpio(st.track)
            info["acVersion"] = limpio(st.acVersion)
            leidos = sum(1 for k in ("carModel", "track", "acVersion") if info[k])
            info["lectura"] = "ok" if leidos == 3 else f"parcial ({leidos} de 3)"
        except Exception as e:
            info["lectura"] = f"error: {e}"
        return info

    def close(self):
        self._ph_mmap.close()
        self._gr_mmap.close()
        if self._st_mmap is not None:
            self._st_mmap.close()


# =============================================================================
# SECCIÓN 2 — TRAZADA Y FRENET (sin cambios)
# =============================================================================

class ReferencePath:
    def __init__(self, csv_path: str, curvature_smooth_m=15.0):
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
        curvature_raw = dtheta / ds

        # Suavizado de curvatura con media movil circular sobre s. La
        # curvatura sale de doble diferencia finita del centro de pista y
        # es ruidosa. Ese ruido entra al volante por el feedforward. La
        # ventana en metros se convierte a numero de puntos.
        self.curvature_raw = curvature_raw
        win = max(1, int(round(curvature_smooth_m / self.avg_spacing)))
        if win % 2 == 0:
            win += 1
        if win >= 3:
            kernel = np.ones(win) / win
            ext = np.concatenate([curvature_raw[-(win // 2):],
                                  curvature_raw,
                                  curvature_raw[:win // 2]])
            self.curvature = np.convolve(ext, kernel, mode="valid")
        else:
            self.curvature = curvature_raw.copy()

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
    def __init__(self, wheelbase_L, steering_ratio_n, Ld_base=2.5, error_decay=0.998,
                 steer_sign=-1, k_ey=0.50, k_ey_v_decay=0.0, Ld_speed_gain=0.6,
                 ff_gain=1.0, theta_clip=1.2):
        self.L = wheelbase_L
        self.n = steering_ratio_n
        self.Ld_base = Ld_base
        self.error_decay = error_decay
        self.steer_sign = steer_sign
        # k_ey         -> peso base del error lateral en el reenganche a la
        #                 linea de referencia.
        # k_ey_v_decay -> reduce el peso efectivo con la velocidad como
        #                 k_ey / (1 + k_ey_v_decay * v). En 0 queda anulado.
        #                 Con ff_gain igual a 1 ya no hay sesgo estructural
        #                 que la realimentacion tenga que compensar, asi que
        #                 no hace falta debilitarla a alta velocidad.
        # Ld_speed_gain-> crecimiento del lookahead con la velocidad.
        # ff_gain      -> fraccion del feedforward kinematico de curvatura.
        #                 Por debajo de 1 la realimentacion tiene que cerrar
        #                 el deficit y para hacerlo necesita que el error
        #                 lateral crezca, lo que produce un error de regimen
        #                 permanente en curva sostenida. En 1 ese error es
        #                 cero por construccion.
        self.k_ey = k_ey
        self.k_ey_v_decay = k_ey_v_decay
        self.Ld_speed_gain = Ld_speed_gain
        self.ff_gain = ff_gain
        self.theta_clip = theta_clip
        # Instrumentacion. Desglose de la ultima referencia generada, para
        # poder separar en el analisis cuanto de la orden de direccion viene
        # de la realimentacion y cuanto del feedforward de curvatura. Sin
        # esta separacion no se distingue una oscilacion del lazo cerrado de
        # un temblor inducido por el ruido de curvatura de la trazada.
        self.last_sat_steps = 0
        self.last_theta0_unclipped = 0.0
        self.last_Ld = 0.0
        self.last_k_ey_eff = 0.0
        self.last_alpha0 = 0.0
        self.last_fb_cmd = 0.0
        self.last_ff_cmd = 0.0
        self.last_kappa0 = 0.0
        self.last_theta_ref_last = 0.0

    def generate(self, e_y, e_psi, path: ReferencePath, idx, speed_ms, N, Ts):
        """
        Los dos términos NO llevan el mismo signo, y ese fue el error que
        vaciaba de sentido toda la referencia en curva.

        `delta_pp0` sale de la geometría de pure pursuit, expresada en el
        convenio de `e_y` y `e_psi`. `delta_ff` es el ángulo de rueda
        cinemático de la trazada, `L` por la curvatura, expresado en el
        convenio del ángulo de rueda. Los dos convenios son opuestos entre
        sí, de modo que aplicarles un único `steer_sign` común deja uno de
        los dos apuntando al revés.

        Verificado sobre la corrida del 2026-09-09 con tres relaciones
        cinemáticas medidas. La derivada de `e_y` vale más 0.978 por
        `v` por el seno de `e_psi` con R² de 0.989. La derivada de `e_psi`
        vale más 0.900 por la diferencia entre la guiñada del vehículo y la
        de la trazada con R² de 0.714. Y la guiñada del vehículo vale más
        0.806 por `delta` por `v` entre `L` con R² de 0.903. Encadenadas
        dicen que un ángulo de rueda positivo hace crecer `e_y`, y que para
        seguir una curvatura positiva hace falta ángulo positivo.

        Contrastado contra lo que hacía el código, la realimentación
        aportaba al comando con correlación de menos 0.774 frente a `e_y`,
        que es lo correcto, mientras el feedforward aportaba con correlación
        de menos 0.998 frente a la curvatura, cuando debía ser positiva. En
        cada curva el feedforward empujaba hacia afuera y la realimentación
        tenía que vencerlo además de trazar, para lo cual el error lateral
        tenía que crecer. De ahí que en recta el seguimiento fuese correcto
        y en curva el vehículo se saliese por fuera de forma sistemática.
        """
        Ld = max(5.0, self.Ld_base + self.Ld_speed_gain * speed_ms)
        k_ey_eff = self.k_ey / (1.0 + self.k_ey_v_decay * max(0.0, speed_ms))
        alpha0 = e_psi + math.atan2(k_ey_eff * e_y, Ld)
        delta_pp0 = math.atan2(2 * self.L * math.sin(alpha0), Ld)
        dist_per_step = max(0.5, speed_ms * Ts)

        theta_ref = np.zeros(N)
        sat_steps = 0
        for k in range(N):
            kappa_k = path.curvature_at_offset(idx, dist_per_step * (k + 1))
            decay = self.error_decay ** k
            # Aportación de cada término AL COMANDO, ya con su signo propio.
            fb_cmd = self.steer_sign * decay * delta_pp0
            ff_cmd = -self.steer_sign * self.ff_gain * self.L * kappa_k
            delta_total = fb_cmd + ff_cmd
            theta_unclipped = delta_total * self.n
            if k == 0:
                self.last_theta0_unclipped = float(theta_unclipped)
                self.last_kappa0 = float(kappa_k)
                self.last_fb_cmd = float(fb_cmd)
                self.last_ff_cmd = float(ff_cmd)
            if abs(theta_unclipped) > self.theta_clip:
                sat_steps += 1
            theta_ref[k] = np.clip(theta_unclipped, -self.theta_clip, self.theta_clip)

        self.last_sat_steps = sat_steps
        self.last_Ld = float(Ld)
        self.last_k_ey_eff = float(k_ey_eff)
        self.last_alpha0 = float(alpha0)
        self.last_theta_ref_last = float(theta_ref[-1])
        return theta_ref


# =============================================================================
# SECCIÓN 3B — PERFIL DE VELOCIDAD (sin cambios; salida en m/s)
# =============================================================================

class SpeedProfileGenerator:
    def __init__(self, path: ReferencePath,
                 ay_max_ms2=6.5, ax_brake_max_ms2=10.25,
                 grip_usage_factor=0.5, grip_usage_brake=None,
                 v_max_recta_ms=50,
                 n_backward_passes=5, profile_smooth_m=25.0):
        """
        grip_usage_brake se separa de grip_usage_factor a propósito.

        El paso hacia atrás que fija dónde empieza a frenar el vehículo debe
        usar la deceleración que el LAZO CERRADO consigue de verdad, no la de
        pico. Medido sobre la corrida del 2026-09-09, con el freno a fondo la
        deceleración instantánea es de 8.43 m/s² de mediana, muy cerca de los
        8.20 que suponía el perfil, pero el lazo cerrado solo llega a freno
        pleno en el 8 por ciento de los ciclos. Promediada por fase de
        frenado completa, incluyendo la rampa del pedal, la deceleración
        efectiva baja a 7.27 m/s² de mediana con percentil 25 en 6.23. El
        perfil planificaba con la capacidad de pico y por eso el vehículo
        llegaba a las curvas 16.9 km/h por encima del objetivo en mediana.

        Si se deja en None se usa grip_usage_factor, que es el
        comportamiento anterior.
        """
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

        # v_limit es el limite fisico local, sin suavizar. Se conserva como
        # referencia de seguridad y para diagnostico.
        self.v_limit = v_max.copy()  # m/s

        # Suavizado opcional del perfil, pero SOLO hacia abajo. Una media
        # movil simetrica sube el perfil en el minimo de cada curva, o sea
        # justo donde el limite de agarre es mas exigente, y ese sobrepaso
        # se traduce en subviraje. Al recortar con el minimo entre el perfil
        # suavizado y el original el filtro solo puede volver mas
        # conservador el objetivo, nunca mas agresivo.
        win = max(1, int(round(self.profile_smooth_m / path.avg_spacing)))
        if win % 2 == 0:
            win += 1
        if win >= 3:
            kernel = np.ones(win) / win
            ext = np.concatenate([v_max[-(win // 2):], v_max, v_max[:win // 2]])
            v_max = np.minimum(np.convolve(ext, kernel, mode="valid"), v_max)

        self.v_max = v_max  # m/s

    def v_ref_min_ahead(self, idx, preview_m):
        """
        Minimo del perfil entre la posicion actual y el punto de preview.

        Leer el perfil EN el punto de preview era el defecto de fondo. A
        mitad de curva ese punto ya cae en la salida, donde la pista admite
        mas velocidad, asi que el objetivo entregado quedaba por encima del
        limite de agarre local y el controlador aceleraba dentro de la
        curva. Tomando el minimo del tramo se conserva el frenado
        anticipado, porque el minimo detecta la curva que viene, y el
        sobrepaso desaparece por construccion.
        """
        steps = max(1, int(round(preview_m / self.path.avg_spacing)))
        window_idx = (idx + np.arange(0, steps + 1)) % self.path.n
        return float(np.min(self.v_max[window_idx]))

    def target_speed_kmh(self, idx, speed_ms, preview_m=20.0,
                         preview_speed_gain=0.35):
        """
        Devuelve UN solo valor objetivo en km/h (no una secuencia): el
        MPCLongitudinalController toma target_speed escalar, no una
        secuencia por paso. Aquí es el propio horizonte interno del
        controlador el que mira hacia adelante con el modelo K/tau. Lo que
        este método aporta es QUÉ velocidad objetivo usar en cada instante.

        La anticipación crece con la velocidad, `preview_m` más
        `preview_speed_gain` por cada m/s, porque la distancia de frenado
        también crece con la velocidad.
        """
        preview_eff = preview_m + preview_speed_gain * max(0.0, speed_ms)
        return self.v_ref_min_ahead(idx, preview_eff) * 3.6


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
        # Instrumentacion del optimizador. Cuando SLSQP no converge, solve
        # reutiliza el warm start anterior y el comando aplicado deja de ser
        # el optimo. Si eso ocurre de forma intermitente produce por si solo
        # un mando irregular, asi que hay que poder descartarlo con datos.
        self.last_ok = True
        self.last_nit = 0
        self.last_cost = 0.0
        self.last_status = 0

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
        self.last_ok = bool(res.success)
        self.last_nit = int(getattr(res, "nit", -1))
        self.last_status = int(getattr(res, "status", -1))
        self.last_cost = float(res.fun) if np.isfinite(res.fun) else float("nan")
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
    def __init__(self, model_params, ts=0.05, horizon=10):
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
        self.last_ok = True
        self.last_nit = 0
        self.last_cost = 0.0

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
        self.last_ok = bool(res.success)
        self.last_nit = int(getattr(res, "nit", -1))
        self.last_cost = float(res.fun) if np.isfinite(res.fun) else float("nan")
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
# SECCIÓN 5B — CONFORMADOR DE PEDAL (acelerador y freno progresivos)
# =============================================================================

class PedalShaper:
    """
    Suaviza el comando longitudinal del MPC antes de mandarlo a vJoy.

    El MPC longitudinal ya penaliza el cambio de u con el termino R_rate,
    pero eso no acota la rampa cuando el objetivo de velocidad salta de
    golpe (entrada a curva, cambio de sector) ni cuando el comando cruza de
    gas a freno. Aqui se impone una rampa explicita. El pedal tarda un
    tiempo minimo fijo en recorrer todo el rango de 0 a 100 por ciento, con
    constantes de subida y de bajada independientes para gas y para freno.

    El estado interno u_shaped vive en el intervalo menos uno a uno, con
    signo positivo para gas y negativo para freno, y es continuo. Al
    cambiar de pedal pasa por cero, de modo que la transicion de gas a
    freno tambien resulta progresiva.
    """
    def __init__(self, ts,
                 throttle_rise_s=0.60, throttle_fall_s=0.35,
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
            # sube el comando: mas gas si u>=0, o soltar freno si u<0
            step = self.d_th_up if u >= 0.0 else self.d_br_dn
            u = min(u_target, u + step)
        elif u_target < u:
            # baja el comando: soltar gas si u>0, o mas freno si u<=0
            step = self.d_th_dn if u > 0.0 else self.d_br_up
            u = max(u_target, u - step)
        self.u = u
        return u

    def reset(self):
        self.u = 0.0


# =============================================================================
# SECCIÓN 5C — RECOLECCIÓN DE DATOS DE LA CORRIDA
# =============================================================================

TELEMETRY_COLUMNS = [
    # --- Tiempo y salud del bucle ---
    "cycle", "t_s", "period_ms", "work_ms", "solve_steer_ms", "solve_speed_ms",
    "packet_id", "packets_repetidos",
    # --- Posición y localización sobre la trazada ---
    "car_x", "car_y", "car_z", "idx", "s_m", "path_x", "path_z",
    "e_y_m", "e_psi_rad", "kappa_path", "kappa_preview",
    # --- Orientación. course es lo que usa el controlador, heading_ac es el
    #     giro real del vehículo. Su diferencia es el ángulo de deriva. ---
    "heading_course_rad", "heading_ac_rad", "sideslip_rad", "yaw_rate_rad_s",
    # --- Velocidad ---
    "v_real_kmh", "v_target_kmh", "v_limit_kmh", "v_from_vector_kmh",
    # --- Desglose de la referencia de dirección ---
    # fb_cmd_rad y ff_cmd_rad son la aportación de cada término AL COMANDO,
    # ya con su signo propio, no los términos crudos. Los paquetes anteriores
    # al 2026-09-09 usaban las columnas delta_pp0_rad y delta_ff0_rad, que
    # guardaban los términos antes de aplicar el signo, y no son comparables.
    "Ld_m", "k_ey_eff", "alpha0_rad", "fb_cmd_rad", "ff_cmd_rad",
    "theta_ref0_deg", "theta_ref_last_deg", "theta_ref0_unclipped_deg",
    "theta_ref_sat_steps",
    # --- Estado y mando de la dirección ---
    "theta_meas_deg", "theta_dot_meas_dps", "tau_Nm", "theta_new_deg",
    "steer_cmd_norm", "steer_out_sat", "steer_angle_ac",
    "mpc_steer_ok", "mpc_steer_nit", "mpc_steer_cost",
    # --- Mando longitudinal ---
    "u_cmd_raw", "u_cmd_deadband", "u_cmd_shaped", "gas_cmd", "brake_cmd",
    "mpc_speed_ok", "mpc_speed_nit", "mpc_speed_cost",
    # --- Lo que Assetto Corsa dice haber recibido y el estado del vehículo ---
    "gas_ac", "brake_ac", "gear", "rpms",
    "accG_x", "accG_y", "accG_z",
    "slip_fl", "slip_fr", "slip_rl", "slip_rr",
    "load_fl", "load_fr", "load_rl", "load_rr",
    "tyres_out", "lap", "is_in_pit",
]


class RunRecorder:
    """
    Recolecta la corrida completa en memoria y la escribe al terminar.

    Se acumula en RAM en vez de escribir cada ciclo porque el periodo de
    control es justamente una de las cosas bajo sospecha, y no conviene
    meter entrada y salida a disco dentro del lazo. El volcado ocurre en el
    bloque finally del bucle, así que también se guarda si la corrida
    termina con Ctrl+C o con una excepción.

    El resultado es una carpeta por corrida con cuatro archivos.
      telemetry.csv        una fila por ciclo, columnas de TELEMETRY_COLUMNS
      reference_profile.csv la trazada y el perfil de velocidad usados
      manifest.json        configuración, versiones, huellas y resumen
      summary.txt          el mismo resumen en texto legible
    """

    def __init__(self, base_dir, cfg, controller_file, track_file):
        self.t_start_wall = time.time()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t_start_wall))
        self.dir = os.path.join(base_dir, f"run_{stamp}")
        os.makedirs(self.dir, exist_ok=True)
        self.rows = []
        self.cfg = dict(cfg)
        self.controller_file = controller_file
        self.track_file = track_file
        self.static_info = {}
        self.notes = []

    def add(self, row):
        self.rows.append(row)

    @staticmethod
    def _sha256(path):
        try:
            h = hashlib.sha256()
            with open(path, "rb") as f:
                for chunk in iter(lambda: f.read(65536), b""):
                    h.update(chunk)
            return h.hexdigest()
        except Exception:
            return None

    def _write_telemetry(self):
        p = os.path.join(self.dir, "telemetry.csv")
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(TELEMETRY_COLUMNS)
            w.writerows(self.rows)
        return p

    def _write_reference(self, path, speed_gen):
        p = os.path.join(self.dir, "reference_profile.csv")
        with open(p, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["idx", "s_m", "x", "z", "heading_rad",
                        "curvature_raw", "curvature_smooth",
                        "v_limit_kmh", "v_max_kmh"])
            for i in range(path.n):
                w.writerow([i, round(float(path.s[i]), 3),
                            round(float(path.x[i]), 4), round(float(path.z[i]), 4),
                            round(float(path.heading[i]), 6),
                            round(float(path.curvature_raw[i]), 8),
                            round(float(path.curvature[i]), 8),
                            round(float(speed_gen.v_limit[i]) * 3.6, 3),
                            round(float(speed_gen.v_max[i]) * 3.6, 3)])
        return p

    def _stats(self):
        """Resumen numérico. Devuelve un diccionario, no imprime nada."""
        if not self.rows:
            return {"ciclos": 0}
        col = {name: i for i, name in enumerate(TELEMETRY_COLUMNS)}

        def num(name):
            v = np.array([r[col[name]] for r in self.rows], dtype=float)
            return v[np.isfinite(v)]

        e_y = num("e_y_m")
        period = num("period_ms")
        theta = num("theta_new_deg")
        cmd = num("steer_cmd_norm")
        ac = num("steer_angle_ac")
        v_real = num("v_real_kmh")
        v_lim = num("v_limit_kmh")
        sat = num("theta_ref_sat_steps")
        tyres = num("tyres_out")
        sside = num("sideslip_rad")

        s = {
            "ciclos": len(self.rows),
            "duracion_s": round(float(num("t_s")[-1]), 2),
            "e_y_abs_medio_m": round(float(np.mean(np.abs(e_y))), 3),
            "e_y_abs_p95_m": round(float(np.percentile(np.abs(e_y), 95)), 3),
            "e_y_abs_max_m": round(float(np.max(np.abs(e_y))), 3),
            "periodo_medio_ms": round(float(np.mean(period)), 2),
            "periodo_p95_ms": round(float(np.percentile(period, 95)), 2),
            "periodo_max_ms": round(float(np.max(period)), 2),
            "ciclos_sobre_periodo_pct": round(
                100.0 * float(np.mean(period > 1.5 * self.cfg.get("Ts_ms", 50.0))), 2),
            "ciclos_mpc_dir_fallo_pct": round(
                100.0 * float(np.mean(num("mpc_steer_ok") < 0.5)), 2),
            "ciclos_mpc_vel_fallo_pct": round(
                100.0 * float(np.mean(num("mpc_speed_ok") < 0.5)), 2),
            "ciclos_theta_ref_recortada_pct": round(100.0 * float(np.mean(sat > 0)), 2),
            "ciclos_eje_vjoy_al_tope_pct": round(
                100.0 * float(np.mean(num("steer_out_sat") > 0.5)), 2),
            "ruedas_fuera_max": int(np.max(tyres)) if tyres.size else 0,
            "ciclos_con_ruedas_fuera_pct": round(100.0 * float(np.mean(tyres > 2)), 2),
            "v_real_max_kmh": round(float(np.max(v_real)), 1),
            "sobrepaso_v_sobre_limite_max_kmh": round(float(np.max(v_real - v_lim)), 1),
            "sideslip_abs_max_deg": round(float(np.degrees(np.max(np.abs(sside)))), 2),
        }

        # Frecuencia dominante del ángulo de dirección. Identifica si el
        # zigzag es un ciclo límite y a qué ritmo ocurre.
        dt = float(np.mean(period)) / 1000.0
        if theta.size > 64 and dt > 1e-4:
            señal = theta - float(np.mean(theta))
            ventana = np.hanning(señal.size)
            esp = np.abs(np.fft.rfft(señal * ventana))
            frec = np.fft.rfftfreq(señal.size, d=dt)
            valido = frec > 0.15
            if np.any(valido):
                k = int(np.argmax(esp[valido]))
                s["frecuencia_dominante_direccion_hz"] = round(
                    float(frec[valido][k]), 3)
        # Cruces por cero de la derivada del ángulo, medida directa del
        # numero de correcciones de volante por segundo.
        if theta.size > 8:
            d = np.diff(theta)
            cruces = int(np.sum(np.diff(np.sign(d)) != 0))
            s["inversiones_volante_por_s"] = round(
                cruces / max(s["duracion_s"], 1e-3), 2)

        # Ajuste entre el comando enviado y el ángulo informado por AC.
        if cmd.size >= 20 and float(np.dot(cmd, cmd)) > 1e-9:
            pend = float(np.dot(cmd, ac) / np.dot(cmd, cmd))
            resid = ac - pend * cmd
            ss_tot = float(np.sum((ac - np.mean(ac)) ** 2))
            s["steer_ac_por_comando_pendiente"] = round(pend, 4)
            s["steer_ac_por_comando_r2"] = round(
                1.0 - float(np.sum(resid ** 2)) / ss_tot, 4) if ss_tot > 1e-9 else None
            s["steer_cmd_norm_rango"] = [round(float(cmd.min()), 4),
                                          round(float(cmd.max()), 4)]
            s["steer_angle_ac_rango"] = [round(float(ac.min()), 4),
                                          round(float(ac.max()), 4)]

        # --- Identificación automática de la cadena de dirección ---
        # Reidentifica en cada corrida los radianes de rueda por unidad de
        # steerAngle, y con ellos la ganancia total de la cadena. Sirve para
        # detectar solo, sin análisis externo, que la calibración dejó de
        # valer porque cambió la configuración del mando, el vehículo o el
        # bloqueo de dirección.
        #
        # Se usa la relación cinemática delta = L * yaw / v, restringida a
        # baja velocidad y con las cuatro ruedas en pista, que es donde el
        # deslizamiento del neumático es pequeño y la relación se cumple.
        try:
            yaw = num("yaw_rate_rad_s")
            vk = num("v_real_kmh")
            fuera = num("tyres_out")
            ey_all = num("e_y_m")
            L_ = float(self.cfg.get("wheelbase_L", 2.7))
            m = ((fuera == 0) & (np.abs(ey_all) < 2.0) & (vk > 20) & (vk < 50)
                 & (np.abs(yaw) > 0.03) & (np.abs(ac) > 0.02))
            if int(m.sum()) >= 40:
                d_kin = L_ * yaw[m] / (vk[m] / 3.6)
                base = float(np.dot(ac[m], ac[m]))
                k_rueda = float(np.dot(ac[m], d_kin) / base)
                resid = d_kin - k_rueda * ac[m]
                var = float(np.sum((d_kin - np.mean(d_kin)) ** 2))
                s["cal_muestras"] = int(m.sum())
                s["cal_rad_rueda_por_steerAngle"] = round(k_rueda, 4)
                s["cal_r2"] = round(1.0 - float(np.sum(resid ** 2)) / var, 4) \
                    if var > 1e-12 else None
                if "steer_ac_por_comando_pendiente" in s:
                    tope = abs(s["steer_ac_por_comando_pendiente"]) * k_rueda
                    s["cal_rad_rueda_eje_completo"] = round(tope, 4)
                    s["cal_grados_rueda_eje_completo"] = round(math.degrees(tope), 2)
                    n_der = float(self.cfg.get("steering_ratio_n_derivada", 0.0))
                    vmax = float(self.cfg.get("vjoy_max_rad", 1.2))
                    if n_der > 0 and vmax > 0:
                        ganancia = (n_der / vmax) * \
                            abs(s["steer_ac_por_comando_pendiente"]) * k_rueda
                        s["cal_ganancia_cadena"] = round(ganancia, 3)
                        s["cal_ganancia_objetivo"] = 1.0
                    esperado = float(self.cfg.get("rueda_rad_eje_completo", 0.0))
                    if esperado > 0:
                        desv = 100.0 * (tope - esperado) / esperado
                        s["cal_desviacion_vs_configurado_pct"] = round(desv, 1)
                        if abs(desv) > 15.0:
                            s["cal_aviso"] = (
                                "la calibracion medida se aleja mas de un 15 por ciento "
                                "del valor configurado en rueda_rad_eje_completo, "
                                "actualizarlo antes de interpretar el resto")
            else:
                s["cal_muestras"] = int(m.sum())
                s["cal_aviso"] = ("muestras insuficientes a baja velocidad para "
                                  "reidentificar la cadena de direccion")
        except Exception as e:
            s["cal_aviso"] = f"no se pudo reidentificar la cadena: {e}"

        # --- Comprobación de signos de la referencia de dirección ---
        # La cadena cinemática dice que un ángulo de rueda positivo hace
        # crecer e_y, y que seguir una curvatura positiva exige ángulo
        # positivo. De ahí salen dos condiciones que deben cumplirse siempre.
        # La aportación de la realimentación al comando tiene que
        # correlacionar NEGATIVAMENTE con e_y, y la del feedforward
        # POSITIVAMENTE con la curvatura de la trazada. Un error de signo en
        # cualquiera de las dos deja el vehículo abriéndose en toda curva
        # mientras en recta parece ir bien, que es difícil de ver a ojo.
        try:
            fb = num("fb_cmd_rad")
            ffw = num("ff_cmd_rad")
            kap = num("kappa_path")
            ey_s = num("e_y_m")
            fuera_s = num("tyres_out")
            m = (fuera_s == 0) & (np.abs(kap) > 0.003)
            if int(m.sum()) >= 50:
                def corr(a, b):
                    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
                        return 0.0
                    return float(np.corrcoef(a, b)[0, 1])
                c_fb = corr(fb[m], ey_s[m])
                c_ff = corr(ffw[m], kap[m])
                s["signo_realim_vs_e_y_corr"] = round(c_fb, 3)
                s["signo_feedfwd_vs_curvatura_corr"] = round(c_ff, 3)
                fallos = []
                if c_fb > -0.2:
                    fallos.append("la realimentacion no se opone al error lateral")
                if c_ff < 0.2:
                    fallos.append("el feedforward no acompana a la curvatura")
                s["signos_referencia"] = "correctos" if not fallos else "; ".join(fallos)
                if fallos:
                    s["signos_aviso"] = ("ERROR DE SIGNO en la referencia de direccion. "
                                         "El vehiculo se abrira en todas las curvas "
                                         "aunque en recta parezca correcto.")
        except Exception:
            pass

        # Ángulo de deriva. Entre `heading` de Assetto Corsa y la dirección
        # del vector velocidad hay un desfase de convención cercano a 90
        # grados, así que la columna cruda no es la deriva. Se estima el
        # desfase con la mediana a velocidad alta y se informa la deriva ya
        # corregida, dejando la columna del registro sin tocar.
        try:
            v_alto = v_real > 60
            if int(v_alto.sum()) >= 50:
                offset = float(np.median(sside[v_alto]))
                corr = (sside - offset + np.pi) % (2 * np.pi) - np.pi
                s["sideslip_offset_convencion_deg"] = round(np.degrees(offset), 2)
                s["sideslip_real_p50_deg"] = round(
                    float(np.degrees(np.median(np.abs(corr[v_alto])))), 2)
                s["sideslip_real_p95_deg"] = round(
                    float(np.degrees(np.percentile(np.abs(corr[v_alto]), 95))), 2)
        except Exception:
            pass
        return s

    def finalize(self, path, speed_gen, static_info=None, extra=None):
        if static_info:
            self.static_info = static_info
        stats = self._stats()
        tel = self._write_telemetry()
        ref = self._write_reference(path, speed_gen)

        manifest = {
            "corrida": os.path.basename(self.dir),
            "inicio_local": time.strftime("%Y-%m-%d %H:%M:%S",
                                          time.localtime(self.t_start_wall)),
            "fin_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            "entorno": {
                "python": sys.version.split()[0],
                "numpy": np.__version__,
                "scipy": scipy_version,
                "plataforma": platform.platform(),
                "procesador": platform.processor(),
                "equipo": platform.node(),
            },
            "archivos": {
                "controlador": os.path.basename(self.controller_file),
                "controlador_sha256": self._sha256(self.controller_file),
                "trazada": os.path.basename(self.track_file),
                "trazada_sha256": self._sha256(self.track_file),
                "telemetria": os.path.basename(tel),
                "referencia": os.path.basename(ref),
            },
            "assetto_corsa": self.static_info,
            "configuracion": self.cfg,
            "resumen": stats,
            "notas": self.notes,
            "columnas_telemetria": TELEMETRY_COLUMNS,
        }
        if extra:
            manifest.update(extra)

        with open(os.path.join(self.dir, "manifest.json"), "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2, ensure_ascii=False)

        lineas = ["RESUMEN DE LA CORRIDA", "=" * 60,
                  f"Carpeta: {self.dir}", ""]
        for k, v in stats.items():
            lineas.append(f"  {k:38s} {v}")
        if self.static_info:
            lineas += ["", "Assetto Corsa:"]
            for k, v in self.static_info.items():
                lineas.append(f"  {k:38s} {v}")
        texto = "\n".join(lineas)
        with open(os.path.join(self.dir, "summary.txt"), "w", encoding="utf-8") as f:
            f.write(texto + "\n")
        return self.dir, texto, stats


# =============================================================================
# SECCIÓN 6 — SALIDA A VJOY (3 ejes: X=steer, Y=gas, RZ=freno)
# =============================================================================

class VJoyOutput:
    def __init__(self, device_id=1):
        self.j = pyvjoy.VJoyDevice(device_id)
        self.center()

    def send_all(self, steer_rad, u_cmd, max_rad=1.2):
        """
        u_cmd viene directo del MPC longitudinal: [-1, 1], + gas, - freno.

        Devuelve gas, freno, el comando de dirección normalizado que de
        verdad salió por el eje X, y un indicador de saturación de ese eje.
        El comando normalizado es la señal que se compara contra
        `steerAngle` de la memoria compartida para reidentificar en cada
        corrida la constante `rueda_rad_eje_completo`, de la que sale la
        relación interna que mantiene la ganancia de la cadena en uno.
        """
        ratio = steer_rad / max_rad
        norm = float(np.clip(ratio, -1.0, 1.0))
        steer_sat = abs(ratio) > 1.0
        steer_val = max(1, min(32767, int((norm + 1.0) / 2.0 * 32766 + 1)))

        if u_cmd >= 0:
            gas_val, brake_val = int(u_cmd * 32767), 0
        else:
            gas_val, brake_val = 0, int(-u_cmd * 32767)

        self.j.data.wAxisX = steer_val
        self.j.data.wAxisY = gas_val
        self.j.data.wAxisZRot = brake_val
        self.j.update()
        return gas_val / 32767.0, brake_val / 32767.0, norm, steer_sat

    def center(self):
        self.j.data.wAxisX = 16384
        self.j.data.wAxisY = 0
        self.j.data.wAxisZRot = 0
        self.j.update()


# =============================================================================
# SECCIÓN 7 — LOOP PRINCIPAL ÚNICO
# =============================================================================

# =============================================================================
# PARÁMETROS EXPUESTOS
# Todo lo que se calibra esta aqui reunido, no disperso por el codigo.
#
# Los valores actuales corresponden al bloque A del diagnostico del
# 2026-09-09, que se apoya en tres medidas hechas sobre monza_fast_lane.csv.
# Primera, leer el perfil de velocidad EN el punto de preview entregaba al
# MPC un objetivo por encima del limite fisico local en el 17.7 por ciento
# del trazado, con un maximo de 122 km/h de exceso, y tomando el minimo del
# tramo ese exceso baja a cero. Segunda, un ff_gain por debajo de 1 produce
# un error lateral de regimen permanente que con 0.55 llegaba a 6 m.
# Tercera, la ventana de suavizado de curvatura de 15 m borraba el 22 por
# ciento del pico de curvatura de las variantes.
# =============================================================================

CFG = {
    # --- Perfil de velocidad y trazada ---
    "grip_usage_factor": 0.80,     # margen de agarre LATERAL
    "grip_usage_brake": 0.60,      # margen de frenado del paso hacia atras. 0.60 sale del
                                   # percentil 25 de la deceleracion que el lazo cerrado
                                   # consigue de verdad, 6.23 m/s2, medida el 2026-09-09
    "curvature_smooth_m": 5.0,     # ventana de suavizado de curvatura. 15 m borraba el pico de las variantes
    "profile_smooth_m": 0.0,       # suavizado del perfil v_max. Solo puede bajarlo, nunca subirlo

    # --- Objetivo de velocidad que recibe el MPC longitudinal ---
    # El objetivo es el MINIMO del perfil entre la posicion actual y el
    # punto de preview, nunca el valor EN el punto de preview.
    "preview_m": 20.0,             # anticipacion base de frenado
    "preview_speed_gain": 0.35,    # anticipacion extra por m/s de velocidad

    # --- Referencia de direccion (pure pursuit + feedforward) ---
    "k_ey": 0.50,                  # peso base del error lateral
    "k_ey_v_decay": 0.0,           # decaimiento del peso lateral con la velocidad. 0 lo anula
    "Ld_speed_gain": 0.6,          # crecimiento del lookahead con la velocidad
    "ff_gain": 1.0,                # feedforward de curvatura completo. Por debajo de 1 hay error permanente

    # --- MPC de direccion ---
    "steer_Rd": 0.6,               # penalizacion de tasa del par. Mas alto, mando mas suave
    "steer_R": 0.02,               # penalizacion de esfuerzo del par

    # --- MPC longitudinal ---
    "long_Q": 6.0,                 # peso de seguimiento de velocidad
    "long_R": 1.5,                 # penalizacion de esfuerzo de pedal
    "long_R_rate": 6.0,            # penalizacion de cambio de pedal
    "speed_deadband_kmh": 3.0,     # banda muerta. Dentro de esto se rueda sin gas ni freno

    # --- Conformador de pedal (rampas 0 a 100 por ciento) ---
    "throttle_rise_s": 0.60,
    "throttle_fall_s": 0.35,
    "brake_rise_s": 0.45,
    "brake_fall_s": 0.30,

    # --- Cadena de dirección, CALIBRADA ---
    # Una sola constante medida sustituye al par steering_ratio_n y
    # vjoy_max_rad que antes se fijaban por separado y sin verificar.
    #
    # rueda_rad_eje_completo son los radianes de rueda directriz que el
    # vehículo toma de verdad cuando el eje de vJoy va a su extremo.
    # Identificado por velocidad de guiñada y confirmado por el ajuste de
    # una circunferencia a tres posiciones de la trayectoria. Una unidad de
    # steerAngle equivale a entre 0.453 y 0.457 rad de rueda, valor estable
    # entre corridas porque es una propiedad del vehículo. El tope es ese
    # factor por el alcance del eje, que depende de la configuración del
    # mando en Assetto Corsa.
    #
    # Historial. Con la casilla de ajuste automático de escala desmarcada el
    # tope era 0.1622 rad, es decir 9.3 grados de rueda. Marcándola pasó a
    # 0.3939 rad, o sea 22.6 grados, que es el valor vigente.
    #
    # Con el par anterior la cadena multiplicaba por 2.15 el ángulo pedido.
    # Ese exceso de ganancia es lo que mantenía el lazo oscilando, y
    # explica por qué ajustar k_ey y ff_gain nunca convergía.
    #
    # HAY QUE VOLVER A MEDIRLA si cambia la configuración del mando en
    # Assetto Corsa, el vehículo o el bloqueo de dirección. El resumen de
    # cada corrida vuelve a identificarla y avisa si se aleja de este valor.
    "rueda_rad_eje_completo": 0.3939,
    "theta_clip_rad": 1.2,
    "vjoy_max_rad": 1.2,
}


def main(use_ffbeast=False):
    print("=" * 60)
    print("  MPC UNIFICADO — Dirección + Gas/Freno | Monza")
    print("=" * 60)

    print("\n[1/6] Cargando trazada...")
    path = ReferencePath("monza_fast_lane.csv",
                         curvature_smooth_m=CFG["curvature_smooth_m"])
    print(f"      {path.n} puntos | {path.total_length:.0f} m ✓")

    print("[2/6] Calculando perfil de velocidad (conservador)...")
    speed_gen = SpeedProfileGenerator(
        path, ay_max_ms2=6.5, ax_brake_max_ms2=10.25,
        grip_usage_factor=CFG["grip_usage_factor"],
        grip_usage_brake=CFG["grip_usage_brake"],
        profile_smooth_m=CFG["profile_smooth_m"])
    print(f"      v_max: {speed_gen.v_max.min()*3.6:.0f}–{speed_gen.v_max.max()*3.6:.0f} km/h ✓")
    print(f"      ay_max={speed_gen.ay_max:.2f} m/s²  "
          f"ax_frenado={speed_gen.ax_brake_max:.2f} m/s² (lazo cerrado, no de pico)")

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
    # La relación interna sale de la constante medida, no se fija a mano.
    # Con esto el ángulo de rueda que el vehículo toma coincide con el que
    # el controlador pide, y la ganancia de la cadena vale uno.
    steering_ratio_n = CFG["vjoy_max_rad"] / CFG["rueda_rad_eje_completo"]
    print(f"      Cadena de dirección: {CFG['rueda_rad_eje_completo']:.4f} rad de rueda "
          f"con el eje a fondo")
    print(f"      steering_ratio_n derivada = {steering_ratio_n:.3f}  "
          f"(antes 16, ganancia de cadena 2.15)")
    print(f"      tope de rueda alcanzable = "
          f"{math.degrees(CFG['rueda_rad_eje_completo']):.1f}°")

    ref_gen = ReferenceGenerator(
        wheelbase_L=2.7, steering_ratio_n=steering_ratio_n, steer_sign=-1,
        k_ey=CFG["k_ey"], k_ey_v_decay=CFG["k_ey_v_decay"],
        Ld_speed_gain=CFG["Ld_speed_gain"], ff_gain=CFG["ff_gain"],
        theta_clip=CFG["theta_clip_rad"])
    mpc_steer = MPCSteeringController(Ts=Ts, N=N_steer,
                                     R=CFG["steer_R"], Rd=CFG["steer_Rd"],
                                     theta_max=CFG["theta_clip_rad"])

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
    mpc_speed = MPCLongitudinalController(model_params, ts=0.05, horizon=10)
    mpc_speed.Q = CFG["long_Q"]
    mpc_speed.R = CFG["long_R"]
    mpc_speed.R_rate = CFG["long_R_rate"]
    pedal = PedalShaper(ts=Ts,
                        throttle_rise_s=CFG["throttle_rise_s"],
                        throttle_fall_s=CFG["throttle_fall_s"],
                        brake_rise_s=CFG["brake_rise_s"],
                        brake_fall_s=CFG["brake_fall_s"])
    print(f"      Dirección: N={N_steer} pasos × {Ts}s = {N_steer*Ts:.2f}s horizonte (τ_dir≈0.2s)")
    print(f"      Velocidad: N={mpc_speed.N} pasos × {mpc_speed.ts}s = "
          f"{mpc_speed.N * mpc_speed.ts:.2f}s horizonte (τ_throttle≈2.81s)")
    print(f"      Objetivo de velocidad por mínimo sobre ventana de preview ✓")

    kf = None
    if use_ffbeast:
        kf = SteeringKalmanFilter(mpc_steer.A, mpc_steer.B,
                                   q_theta=1e-6, q_theta_dot=1e-2, r_theta=1e-4)

    # --- Recolección de la corrida ---
    cfg_corrida = dict(CFG)
    cfg_corrida.update({
        "Ts_s": Ts, "Ts_ms": Ts * 1000.0, "N_steer": N_steer,
        "steering_ratio_n_derivada": steering_ratio_n,
        "wheelbase_L": 2.7, "steer_sign": -1,
        "long_horizon": mpc_speed.N, "long_ts": mpc_speed.ts,
        "modelo_longitudinal": model_params,
        "planta_direccion": "FFBeast" if use_ffbeast else "simulada",
        "ay_max_ms2": 6.5, "ax_brake_max_ms2": 10.25,
    })
    recorder = RunRecorder(
        base_dir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs"),
        cfg=cfg_corrida,
        controller_file=os.path.abspath(__file__),
        track_file=os.path.abspath("monza_fast_lane.csv"))
    static_info = sm.read_static_info()
    print(f"      Corrida: {recorder.dir}")
    print(f"      Assetto Corsa: coche={static_info.get('carModel')} "
          f"pista={static_info.get('track')} (lectura {static_info.get('lectura')})")

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
    t_prev_loop = t0
    packets_repetidos = 0
    heading_ac_prev = None
    t_heading_prev = t0

    print(f"{'t(s)':>6} | {'e_y(m)':>7} | {'θ(°)':>7} | {'v(km/h)':>8} | "
          f"{'v_obj(km/h)':>11} | {'gas':>5} | {'brake':>5} | flags")
    print("  SAT = referencia de dirección recortada por el tope")
    print("  OUT = más de dos ruedas fuera de pista")
    print("-" * 90)

    try:
        while True:
            t_loop = time.time()
            gr = sm.read_graphics()
            ph = sm.read_physics()

            if gr.packetId == last_packet_id:
                packets_repetidos += 1
                time.sleep(0.005)
                continue
            last_packet_id = gr.packetId

            # Periodo real entre ciclos de control, que no es lo mismo que
            # el tiempo de trabajo dentro del ciclo.
            period_ms = (t_loop - t_prev_loop) * 1000.0
            t_prev_loop = t_loop

            car_x, car_y, car_z = gr.carCoordinates
            speed_ms = ph.speedKmh / 3.6
            speed_kmh = ph.speedKmh

            car_vx, car_vy, car_vz = ph.velocity
            if speed_ms > MIN_SPEED_MS:
                car_heading = math.atan2(car_vz, car_vx)
                last_heading = car_heading
            else:
                car_heading = last_heading

            # Orientación real del vehículo frente a la dirección del vector
            # velocidad. El controlador usa la segunda. La diferencia es el
            # ángulo de deriva y se registra sin alterar el control, para
            # poder comprobar después si introduce retardo de fase.
            heading_ac = float(ph.heading)
            sideslip = (car_heading - heading_ac + math.pi) % (2 * math.pi) - math.pi
            if heading_ac_prev is None:
                yaw_rate = 0.0
            else:
                d_head = (heading_ac - heading_ac_prev + math.pi) % (2 * math.pi) - math.pi
                yaw_rate = d_head / max(1e-3, t_loop - t_heading_prev)
            heading_ac_prev = heading_ac
            t_heading_prev = t_loop

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
            # El recorte de velocidad queda solo por curvatura de pista, que
            # ya vive en speed_gen.v_max. No se recorta por e_y para no
            # acoplar la oscilacion lateral al lazo longitudinal.
            v_target_kmh = speed_gen.target_speed_kmh(
                idx, speed_ms,
                preview_m=CFG["preview_m"],
                preview_speed_gain=CFG["preview_speed_gain"])
            v_limit_kmh = float(speed_gen.v_limit[idx]) * 3.6
            t_speed0 = time.time()
            u_cmd = mpc_speed.compute_control(speed_kmh, v_target_kmh)
            t_speed_ms = (time.time() - t_speed0) * 1000.0
            u_cmd_raw = u_cmd

            # Banda muerta: dentro de +-speed_deadband_kmh se rueda sin gas
            # ni freno, elimina el hunting fino a mitad de curva.
            if abs(speed_kmh - v_target_kmh) < CFG["speed_deadband_kmh"]:
                u_cmd = 0.0

            u_cmd_shaped = pedal.shape(u_cmd)
            # Cierra el lazo del conformador con el MPC: el MPC toma como
            # comando previo el que de verdad se aplico, no el que el pidio,
            # asi no pelea contra el retardo de su propio actuador.
            mpc_speed.u_prev = u_cmd_shaped

            # --- Salida conjunta ---
            gas, brake, steer_cmd_norm, steer_out_sat = vjoy.send_all(
                theta_new, u_cmd_shaped, max_rad=CFG["vjoy_max_rad"])

            # --- Lectura del estado del vehículo para el registro ---
            steer_angle_ac = float(ph.steerAngle)
            acc_x, acc_y, acc_z = ph.accG
            slip = list(ph.wheelSlip)
            load = list(ph.wheelLoad)
            tyres_out = int(ph.numberOfTyresOut)

            cycle += 1
            t_elapsed = time.time() - t0
            work_ms = (time.time() - t_loop) * 1000.0

            if cycle % 10 == 0:
                flags = ("SAT" if ref_gen.last_sat_steps > 0 else "   ")
                flags += (" OUT" if tyres_out > 2 else "    ")
                flags += ("" if mpc_steer.last_ok else " NOCONV")
                print(f"{t_elapsed:>6.1f} | {e_y:>7.2f} | {math.degrees(theta_new):>7.1f} | "
                      f"{speed_kmh:>8.1f} | {v_target_kmh:>11.1f} | {gas:>5.2f} | {brake:>5.2f} | "
                      f"{flags} | solve: dir={t_steer_ms:.1f}ms vel={t_speed_ms:.1f}ms "
                      f"periodo={period_ms:.1f}ms")

            recorder.add([
                cycle, round(t_elapsed, 4), round(period_ms, 3), round(work_ms, 3),
                round(t_steer_ms, 3), round(t_speed_ms, 3),
                int(gr.packetId), packets_repetidos,
                round(float(car_x), 4), round(float(car_y), 4), round(float(car_z), 4),
                int(idx), round(float(s), 3),
                round(float(path.x[idx]), 4), round(float(path.z[idx]), 4),
                round(float(e_y), 5), round(float(e_psi), 6),
                round(float(path.curvature[idx]), 8), round(ref_gen.last_kappa0, 8),
                round(float(car_heading), 6), round(heading_ac, 6),
                round(float(sideslip), 6), round(float(yaw_rate), 5),
                round(float(speed_kmh), 3), round(float(v_target_kmh), 3),
                round(float(v_limit_kmh), 3),
                round(3.6 * math.sqrt(car_vx ** 2 + car_vy ** 2 + car_vz ** 2), 3),
                round(ref_gen.last_Ld, 3), round(ref_gen.last_k_ey_eff, 5),
                round(ref_gen.last_alpha0, 6), round(ref_gen.last_fb_cmd, 6),
                round(ref_gen.last_ff_cmd, 6),
                round(math.degrees(theta_ref_seq[0]), 4),
                round(math.degrees(ref_gen.last_theta_ref_last), 4),
                round(math.degrees(ref_gen.last_theta0_unclipped), 4),
                int(ref_gen.last_sat_steps),
                round(math.degrees(theta_current), 4),
                round(math.degrees(theta_dot_current), 4),
                round(float(tau), 5), round(math.degrees(theta_new), 4),
                round(float(steer_cmd_norm), 6), int(steer_out_sat),
                round(steer_angle_ac, 6),
                int(mpc_steer.last_ok), int(mpc_steer.last_nit),
                round(float(mpc_steer.last_cost), 6),
                round(float(u_cmd_raw), 5), round(float(u_cmd), 5),
                round(float(u_cmd_shaped), 5),
                round(float(gas), 5), round(float(brake), 5),
                int(mpc_speed.last_ok), int(mpc_speed.last_nit),
                round(float(mpc_speed.last_cost), 6),
                round(float(ph.gas), 5), round(float(ph.brake), 5),
                int(ph.gear), int(ph.rpms),
                round(float(acc_x), 5), round(float(acc_y), 5), round(float(acc_z), 5),
                round(float(slip[0]), 5), round(float(slip[1]), 5),
                round(float(slip[2]), 5), round(float(slip[3]), 5),
                round(float(load[0]), 3), round(float(load[1]), 3),
                round(float(load[2]), 3), round(float(load[3]), 3),
                tyres_out, int(gr.completedLaps), int(gr.isInPit),
            ])
            packets_repetidos = 0

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
        try:
            static_info = sm.read_static_info()
        except Exception:
            static_info = {}
        sm.close()
        print(f"\nCiclos ejecutados: {cycle}")
        if cycle > 0:
            carpeta, texto, _ = recorder.finalize(path, speed_gen, static_info)
            print()
            print(texto)
            print()
            print("=" * 60)
            print(f"  Corrida guardada en: {carpeta}")
            print("  Comprime esa carpeta completa y súbela para el análisis.")
            print("=" * 60)
        else:
            print("No se registró ningún ciclo, no se guarda la corrida.")


if __name__ == "__main__":
    import sys
    use_ffbeast = "--ffbeast" in sys.argv
    main(use_ffbeast=use_ffbeast)
