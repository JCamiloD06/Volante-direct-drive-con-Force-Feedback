# =============================================================================
# mpc_monza_FINAL.py
# MPC real (horizonte + optimización) para conducción autónoma en Monza — Assetto Corsa
#
# ARQUITECTURA:
#   1. Lee posición real del carro via SDK oficial de AC (carCoordinates)
#   2. Calcula error lateral e_y y error de heading e_psi (coordenadas Frenet)
#   3. Genera una PREVIEW de theta_ref sobre un horizonte N usando la curvatura
#      futura de la trazada
#   4. El MPC resuelve un QP sobre ese horizonte y aplica solo el primer torque
#      (receding horizon), respetando límites de torque y de tasa de cambio
#   5. El torque se aplica a una "planta" abstracta:
#        - SimulatedSteeringPlant  → Fase A: no hay volante físico, se simula
#          la dinámica J·θ̈ = τ - b·θ̇ y el resultado se manda a vJoy
#        - ODriveSteeringPlant     → Fase B: el ODrive real mueve el motor y
#          el encoder real es la fuente de verdad del estado (θ, θ̇)
#      En ambos casos, θ resultante se manda a vJoy para mover el carro en AC
#
# REQUISITOS:
#   pip install numpy scipy pyvjoy
#   (Fase B) pip install pysdl2 pysdl2-dll
#   Assetto Corsa corriendo en Monza con sesión activa en pista
#   vJoy Device 1 configurado como steering en AC
#
# ARCHIVOS NECESARIOS en la misma carpeta:
#   monza_fast_lane.csv   (trazada de Monza extraída del fast_lane.ai)
#
# NOTA IMPORTANTE SOBRE HARDWARE:
#   El MKS ODrive Mini de este proyecto corre firmware FFBeast, NO firmware
#   nativo de ODrive. FFBeast expone el volante a Windows como un dispositivo
#   HID de Force Feedback (DirectInput), no como un dispositivo ODrive por
#   protocolo serial. Por eso el control de torque aquí se hace mandando un
#   efecto "Constant Force" vía SDL2 haptics (SDL_HapticUpdateEffect), no
#   con `odrive.axis0.controller.input_torque` como en un ODrive real.
#
#   La magnitud que le mandas a SDL2 (rango ±32767) NO es Nm directo — es un
#   valor normalizado que FFBeast escala internamente según el límite de
#   fuerza que configuraste en su app. Tienes que calibrar tú el factor
#   NM_TO_MAGNITUDE (ver sección de calibración) antes de confiar en que
#   tau_max del MPC corresponde a un torque físico real y seguro.
#
#   Con vJoy manteniéndose como intermediario (decisión de este proyecto):
#   AC sigue leyendo el eje virtual de vJoy, NO el eje físico de FFBeast
#   directamente. El flujo es: MPC calcula τ → FFBeast aplica ese torque al
#   motor → se LEE la posición real resultante del volante físico (vía SDL2
#   joystick axis, no un valor simulado) → esa posición real se manda a
#   vJoy → AC mueve el carro según lo que el volante físico realmente hizo.
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
# SECCIÓN 1 — SDK OFICIAL DE ASSETTO CORSA (Shared Memory)
# =============================================================================

class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId",            ctypes.c_int32),
        ("gas",                 ctypes.c_float),
        ("brake",                ctypes.c_float),
        ("fuel",                 ctypes.c_float),
        ("gear",                 ctypes.c_int32),
        ("rpms",                 ctypes.c_int32),
        ("steerAngle",           ctypes.c_float),
        ("speedKmh",             ctypes.c_float),
        ("velocity",             ctypes.c_float * 3),
        ("accG",                 ctypes.c_float * 3),
        ("wheelSlip",            ctypes.c_float * 4),
        ("wheelLoad",            ctypes.c_float * 4),
        ("wheelsPressure",       ctypes.c_float * 4),
        ("wheelAngularSpeed",    ctypes.c_float * 4),
        ("tyreWear",             ctypes.c_float * 4),
        ("tyreDirtyLevel",       ctypes.c_float * 4),
        ("tyreCoreTemperature",  ctypes.c_float * 4),
        ("camberRAD",            ctypes.c_float * 4),
        ("suspensionTravel",     ctypes.c_float * 4),
        ("drs",                  ctypes.c_float),
        ("tc",                   ctypes.c_float),
        ("heading",               ctypes.c_float),
        ("pitch",                 ctypes.c_float),
        ("roll",                  ctypes.c_float),
        ("cgHeight",              ctypes.c_float),
        ("carDamage",             ctypes.c_float * 5),
        ("numberOfTyresOut",      ctypes.c_int32),
        ("pitLimiterOn",          ctypes.c_int32),
        ("abs",                   ctypes.c_float),
    ]

class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId",              ctypes.c_int32),
        ("status",                ctypes.c_int32),
        ("session",               ctypes.c_int32),
        ("currentTime",           ctypes.c_wchar * 15),
        ("lastTime",              ctypes.c_wchar * 15),
        ("bestTime",              ctypes.c_wchar * 15),
        ("split",                 ctypes.c_wchar * 15),
        ("completedLaps",         ctypes.c_int32),
        ("position",              ctypes.c_int32),
        ("iCurrentTime",          ctypes.c_int32),
        ("iLastTime",             ctypes.c_int32),
        ("iBestTime",             ctypes.c_int32),
        ("sessionTimeLeft",       ctypes.c_float),
        ("distanceTraveled",      ctypes.c_float),
        ("isInPit",               ctypes.c_int32),
        ("currentSectorIndex",    ctypes.c_int32),
        ("lastSectorTime",        ctypes.c_int32),
        ("numberOfLaps",          ctypes.c_int32),
        ("tyreCompound",          ctypes.c_wchar * 33),
        ("replayTimeMultiplier",  ctypes.c_float),
        ("normalizedCarPosition", ctypes.c_float),
        ("carCoordinates",        ctypes.c_float * 3),
    ]

class ACSharedMemory:
    def __init__(self):
        self._ph_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysics),  "acpmf_physics")
        self._gr_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFileGraphic), "acpmf_graphics")

    def read_physics(self)  -> SPageFilePhysics:
        return SPageFilePhysics.from_buffer_copy(self._ph_mmap)

    def read_graphics(self) -> SPageFileGraphic:
        return SPageFileGraphic.from_buffer_copy(self._gr_mmap)

    def close(self):
        self._ph_mmap.close()
        self._gr_mmap.close()


# =============================================================================
# SECCIÓN 2 — TRAZADA DE MONZA Y LOCALIZADOR (Frenet)
# =============================================================================

class ReferencePath:
    """
    Carga la trazada de Monza del CSV y provee:
      - Proyección del carro sobre la trazada (error lateral e_y, heading e_psi)
      - Curvatura futura para el feedforward del MPC
    """

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

        # Heading (tangente) en cada punto
        dx = np.roll(self.x, -1) - self.x
        dz = np.roll(self.z, -1) - self.z
        self.heading = np.arctan2(dz, dx)

        # Curvatura: dθ/ds
        dtheta = np.roll(self.heading, -1) - self.heading
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        next_x  = np.roll(self.x, -1)
        next_z  = np.roll(self.z, -1)
        ds      = np.sqrt((next_x - self.x)**2 + (next_z - self.z)**2)
        ds[ds < 1e-3] = 1e-3
        self.curvature = dtheta / ds

        self._last_idx = 0

    def nearest(self, car_x, car_z, window=60):
        idxs = (self._last_idx + np.arange(-window, window)) % self.n
        d2   = (self.x[idxs] - car_x)**2 + (self.z[idxs] - car_z)**2
        best = idxs[np.argmin(d2)]
        if d2.min() > 400:  # salto grande → búsqueda global
            d2g  = (self.x - car_x)**2 + (self.z - car_z)**2
            best = int(np.argmin(d2g))
        self._last_idx = int(best)
        return self._last_idx

    def frenet(self, car_x, car_z, car_heading):
        """
        Retorna (e_y, e_psi, s, idx)
          e_y   > 0  → carro a la izquierda de la referencia
          e_psi = heading_carro - heading_pista  ∈ [-π, π]
        """
        idx = self.nearest(car_x, car_z)
        ph  = self.heading[idx]
        dx  = car_x - self.x[idx]
        dz  = car_z - self.z[idx]
        e_y = -math.sin(ph) * dx + math.cos(ph) * dz
        e_psi = (car_heading - ph + math.pi) % (2*math.pi) - math.pi
        return e_y, e_psi, self.s[idx], idx

    def curvature_at_offset(self, idx, dist_m):
        """Curvatura en la trazada a `dist_m` metros adelante del idx dado."""
        idx_step = max(1, round(dist_m / self.avg_spacing))
        return float(self.curvature[(idx + idx_step) % self.n])


# =============================================================================
# SECCIÓN 3 — GENERADOR DE REFERENCIA (Pure Pursuit + preview de curvatura)
# =============================================================================

class ReferenceGenerator:
    """
    Genera la trayectoria de referencia theta_ref[0..N-1] que el MPC debe
    seguir sobre su horizonte de predicción.

    - Paso k=0: corrección completa de Pure Pursuit (e_y, e_psi medidos)
    - Pasos k>0: no tenemos una medición futura real de e_y/e_psi (eso
      requeriría un modelo del vehículo completo, no solo del volante), así
      que se usa el feedforward de curvatura de la pista en el punto donde
      se espera que esté el carro, MÁS la corrección de error actual con
      un decaimiento MUY suave (error_decay cercano a 1). El decaimiento
      es leve a propósito: el horizonte del volante es corto (0.5s), y si
      la corrección decae demasiado rápido, el MPC termina priorizando el
      objetivo casi-cero del final del horizonte (sobre todo si Qf pesa
      más que Q) y deja de corregir errores reales y sostenidos — ese fue
      exactamente el bug detectado en pruebas: con decay=0.75 y Qf=2*Q, el
      volante se quedaba estancado muy por debajo del ángulo necesario
      para corregir un error lateral grande y persistente.
    """

    def __init__(self, wheelbase_L, steering_ratio_n, Ld_base=2.5, error_decay=0.998,
                 steer_sign=-1):
        """
        steer_sign: +1 o -1. Existe porque el sistema de coordenadas de AC
        (motor tipo DirectX, mano izquierda) puede no coincidir con la
        convención matemática estándar (mano derecha) que usa la fórmula
        de e_y en ReferencePath.frenet(). Si el signo está mal, el sistema
        es internamente consistente (no tira errores ni NaN) pero corrige
        en la dirección EQUIVOCADA: el error crece en vez de achicarse y
        el volante se satura casi de inmediato hacia el límite — exactamente
        el síntoma de un lazo de realimentación con signo invertido.
        Por defecto queda en -1 como mejor estimación inicial; si el
        comportamiento sigue divergiendo, prueba con +1.
        """
        self.L = wheelbase_L
        self.n = steering_ratio_n
        self.Ld_base = Ld_base
        self.error_decay = error_decay
        self.steer_sign = steer_sign

    def generate(self, e_y, e_psi, path: ReferencePath, idx, speed_ms, N, Ts):
        Ld = max(5.0, self.Ld_base + 0.5 * speed_ms)
        alpha0 = e_psi + math.atan2(0.3 * e_y, Ld)
        delta_pp0 = math.atan2(2 * self.L * math.sin(alpha0), Ld)

        dist_per_step = max(0.5, speed_ms * Ts)  # avanza al menos algo aunque esté casi parado

        theta_ref = np.zeros(N)
        for k in range(N):
            kappa_k = path.curvature_at_offset(idx, dist_per_step * (k + 1))
            delta_ff = self.L * kappa_k

            decay = self.error_decay ** k
            delta_total = self.steer_sign * (decay * delta_pp0 + 0.5 * delta_ff)

            # Rango unificado con MPCSteeringController.theta_max y
            # VJoyOutput.max_rad (450 grados de rotación total del volante)
            theta_k = np.clip(delta_total * self.n, -7.85, 7.85)
            theta_ref[k] = theta_k

        return theta_ref


# =============================================================================
# SECCIÓN 4 — MPC REAL (horizonte + QP) PARA EL VOLANTE
# =============================================================================

class MPCSteeringController:
    """
    MPC de horizonte finito para el volante.

    Modelo (discreto, doble integrador con fricción viscosa):
      x = [theta, theta_dot]
      x_{k+1} = A x_k + B u_k
      A = [[1, Ts], [0, 1 - Ts*b/J]]
      B = [[0], [Ts/J]]

    Costo sobre el horizonte N:
      J = sum_k Q*(theta_k - theta_ref_k)^2
        + Qf*(theta_N - theta_ref_N)^2         (peso extra al último paso)
        + R*u_k^2
        + Rd*(u_k - u_{k-1})^2

    Restricciones:
      |u_k|          <= tau_max
      |u_k - u_{k-1}| <= rate_max

    Se resuelve el QP completo cada ciclo (receding horizon) y solo se aplica
    u_0*. Esto SÍ es un MPC real: predice N pasos, optimiza sobre esa
    secuencia completa y descarta el resto.
    """

    def __init__(self, J=0.08, b=0.05, Ts=0.05, N=10,
                 Q=10.41, Qf=10.41, R=0.0, Rd=0.288,
                 tau_max=2.0, rate_max=1.0, theta_max=7.85):
        self.J, self.b, self.Ts, self.N = J, b, Ts, N
        self.Q, self.Qf, self.R, self.Rd = Q, Qf, R, Rd
        self.tau_max, self.rate_max, self.theta_max = tau_max, rate_max, theta_max

        self.A = np.array([[1.0, Ts],
                            [0.0, 1.0 - Ts * b / J]])
        self.B = np.array([0.0, Ts / J])

        self.u_prev = 0.0
        self._u_warm = np.zeros(N)  # warm start para el optimizador

    def reset(self):
        """
        Reinicia el estado interno del optimizador (u_prev y el warm-start).
        Llamar cuando el control se desactiva o el auto está parado, para que
        al reengancharse no arrastre un u_prev/_u_warm de antes de la pausa
        (evita saltos de torque o un rate_constraint artificialmente
        restrictivo justo al reanudar el control).
        """
        self.u_prev = 0.0
        self._u_warm = np.zeros(self.N)

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
        """
        Resuelve el QP del horizonte y retorna:
          tau_apply   → torque a aplicar este ciclo (u_0*)
          theta_pred  → theta predicho en el siguiente paso (para logging)
        """
        x0 = np.array([theta_current, theta_dot_current])

        bounds = [(-self.tau_max, self.tau_max)] * self.N

        def rate_constraint(u_seq):
            du = np.diff(np.concatenate(([self.u_prev], u_seq)))
            return self.rate_max - np.abs(du)  # >= 0

        def theta_bound_constraint(u_seq):
            xs = self._predict(x0, u_seq)
            return self.theta_max - np.abs(xs[1:, 0])  # >= 0

        constraints = [
            {"type": "ineq", "fun": rate_constraint},
            {"type": "ineq", "fun": theta_bound_constraint},
        ]

        res = minimize(
            self._cost, self._u_warm, args=(x0, theta_ref_seq, self.u_prev),
            method="SLSQP", bounds=bounds, constraints=constraints,
            options={"maxiter": 40, "ftol": 1e-6},
        )

        if res.success:
            u_opt = res.x
        else:
            # Fallback seguro: mantener el último torque, no dejar el volante
            # sin comando ni aplicar algo no verificado por el solver
            u_opt = self._u_warm

        tau_apply = float(np.clip(u_opt[0], -self.tau_max, self.tau_max))
        # Warm start para el próximo ciclo: desplaza la solución un paso
        self._u_warm = np.concatenate([u_opt[1:], [u_opt[-1]]])
        self.u_prev = tau_apply

        x1 = self.A @ x0 + self.B * tau_apply
        return tau_apply, x1[0], x1[1]


# =============================================================================
# SECCIÓN 4B — ESTIMADOR DE ESTADO (FILTRO DE KALMAN)
# =============================================================================

class SteeringKalmanFilter:
    """
    Filtro de Kalman para estimar el estado del volante [theta, theta_dot]
    a partir de UNA sola medición ruidosa: theta, leído del eje del
    joystick/HID en Fase B. theta_dot NO se mide directamente — antes se
    calculaba en FFBeastSteeringPlant.get_state() por diferenciación
    numérica cruda ((theta - theta_prev) / dt), lo cual amplifica el ruido
    de cuantización del eje. Aquí en cambio theta_dot se infiere a partir
    del modelo, con la medición de theta corrigiendo esa inferencia cada
    ciclo.

    Usa el MISMO modelo (A, B) que ya usa el MPC internamente para predecir
    (doble integrador con fricción viscosa: J·θ̈ = τ - b·θ̇), así que el
    estimador y el controlador comparten la misma noción de la dinámica del
    volante por construcción — evita el bug clásico de un observador que
    predice con un modelo distinto al que usa el controlador para optimizar.

    Ciclo estándar de Kalman (predicción + corrección):
      Predicción:  x_pred = A x + B u
                   P_pred = A P A^T + Q_proc
      Corrección:  K      = P_pred H^T (H P_pred H^T + R_meas)^-1
                   x       = x_pred + K (z - H x_pred)
                   P       = (I - K H) P_pred

    H = [1, 0] porque SOLO theta es medible (el eje del joystick); theta_dot
    es un estado latente que el filtro infiere, no una medición.
    """

    def __init__(self, A, B, q_theta=1e-6, q_theta_dot=1e-2, r_theta=1e-4,
                 theta0=0.0, theta_dot0=0.0):
        """
        q_theta, q_theta_dot: ruido de proceso — cuánto confías en el
          modelo J·θ̈=τ-b·θ̇. Súbelos si el volante hace cosas que ese
          modelo no predice bien (fricción seca, backlash, golpes de la
          pista vía el FFB, etc). q_theta_dot suele necesitar ser bastante
          mayor que q_theta porque theta_dot es lo más difícil de modelar.
        r_theta: ruido de medición — cuánto ruido/jitter tiene el eje HID.
          Súbelo si ves que theta medido salta/tiembla más de lo esperado;
          un r_theta más alto hace que el filtro confíe más en el modelo
          y menos en cada lectura individual (más suavizado, más lag).
        Estos tres parámetros son justo lo que hay que CALIBRAR empíricamente
        con el hardware real, igual que nm_to_magnitude en FFBeastSteeringPlant
        — no hay valores "correctos" universales.
        """
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float).reshape(2)
        self.H = np.array([[1.0, 0.0]])
        self.Q = np.diag([q_theta, q_theta_dot])
        self.R = np.array([[r_theta]])
        self.x = np.array([theta0, theta_dot0], dtype=float)
        self.P = np.eye(2) * 1e-3

    def predict(self, u):
        """Paso de predicción usando el último torque aplicado (u_{k-1})."""
        self.x = self.A @ self.x + self.B * u
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, theta_meas):
        """Corrige la predicción con la medición real de theta de este ciclo."""
        z = np.array([theta_meas])
        y = z - self.H @ self.x                       # innovación
        S = self.H @ self.P @ self.H.T + self.R        # covarianza de innovación
        K = (self.P @ self.H.T @ np.linalg.inv(S)).flatten()  # ganancia de Kalman
        self.x = self.x + K * y[0]
        self.P = (np.eye(2) - np.outer(K, self.H.flatten())) @ self.P

    def state(self):
        return float(self.x[0]), float(self.x[1])

    def reset(self, theta0=0.0, theta_dot0=0.0):
        self.x = np.array([theta0, theta_dot0], dtype=float)
        self.P = np.eye(2) * 1e-3


# =============================================================================
# SECCIÓN 5 — PLANTAS DEL VOLANTE (simulada vs. física ODrive)
# =============================================================================

class SimulatedSteeringPlant:
    """
    Fase A: no hay volante físico. Se simula la dinámica J·θ̈ = τ - b·θ̇
    integrándola con el mismo modelo que usa el MPC internamente, y el
    resultado (theta) es lo que se manda a vJoy.
    Esta es la ÚNICA fuente de verdad del estado en Fase A — evita el error
    anterior de usar ph.steerAngle (que viene del propio AC) como feedback
    de un volante que en Fase A ni siquiera existe físicamente.
    No necesita SteeringKalmanFilter: theta y theta_dot ya son exactos por
    construcción (los integra la propia planta), no hay medición ruidosa
    que filtrar.
    """

    def __init__(self, J=0.08, b=0.05, Ts=0.05):
        self.J, self.b, self.Ts = J, b, Ts
        self.theta = 0.0
        self.theta_dot = 0.0

    def get_state(self):
        return self.theta, self.theta_dot

    def apply_torque(self, tau):
        theta_ddot = (tau - self.b * self.theta_dot) / self.J
        self.theta_dot += theta_ddot * self.Ts
        self.theta += self.theta_dot * self.Ts
        # Rango unificado con MPCSteeringController.theta_max y
        # VJoyOutput.max_rad (450 grados de rotación total del volante)
        self.theta = float(np.clip(self.theta, -7.85, 7.85))
        return self.theta, self.theta_dot

    def center(self):
        self.theta = 0.0
        self.theta_dot = 0.0

    def close(self):
        pass


class FFBeastSteeringPlant:
    """
    Fase B: el volante físico (motor de hoverboard + MKS con firmware
    FFBeast) es la planta real. El volante se ve ante Windows como un
    dispositivo HID de Force Feedback (DirectInput), así que:
      - El torque se manda como un efecto "Constant Force" vía SDL2 haptics,
        actualizando su magnitud cada ciclo (SDL_HapticUpdateEffect).
      - La posición real (theta) se lee del eje del joystick vía SDL2 — es
        la ÚNICA medición real disponible.
      - theta_dot ya NO se estima aquí por diferenciación numérica cruda
        (ese cálculo vivía antes en get_state() y amplificaba el jitter de
        cuantización del eje HID). Se sigue devolviendo por compatibilidad/
        logging, pero el loop principal usa SteeringKalmanFilter para
        obtener el theta_dot que realmente alimenta al MPC — ver SECCIÓN 4B.

    IMPORTANTE — calibra ANTES de correr con el motor conectado:
      - max_steering_rad: la mitad del rango total de rotación que
        configuraste en la app de FFBeast (ej. si configuraste 900° de
        rotación total, max_steering_rad = radians(450)).
      - nm_to_magnitude: factor que convierte Nm "deseados" por el MPC a
        la magnitud SDL2 (±32767) que realmente le pega al límite de fuerza
        configurado en FFBeast. Esto NO es un valor físico real, es una
        aproximación que TÚ calibras empíricamente (ver calibrate_ffbeast()
        más abajo) — no confíes en que tau_max del MPC = tau_max real del
        motor hasta haber hecho esa calibración.
      - direction_sign: algunos dispositivos FFBeast invierten el sentido
        del torque respecto al signo esperado. Verifica con un torque
        pequeño positivo si el volante gira hacia el lado correcto.
      - Prueba SIEMPRE primero con el volante desacoplado del eje / sin
        carga, y con una forma fácil de cortar la corriente (botón de
        emergencia físico, no solo Ctrl+C).
    """

    def __init__(self, max_steering_rad=math.radians(450), tau_max_motor=2.0,
                 direction_sign=1, joystick_index=None, axis_index=0):
        if not SDL2_AVAILABLE:
            raise RuntimeError(
                "El paquete 'sdl2' (PySDL2) no está instalado. "
                "Instala con: pip install pysdl2 pysdl2-dll"
            )

        self.max_steering_rad = max_steering_rad
        self.tau_max_motor = tau_max_motor
        # Mapeo directo: el rango completo de SDL2 (±32767) corresponde
        # exactamente a ±tau_max_motor. Esto SOLO es correcto si el límite
        # de fuerza configurado dentro de la app de FFBeast también
        # corresponde a tau_max_motor Nm reales — si en FFBeast dejaste un
        # límite de fuerza distinto (más alto o más bajo), esta relación
        # se rompe y hay que recalibrar. Verifícalo con --calibrate antes
        # de confiar en este número.
        self.nm_to_magnitude = 32767.0 / tau_max_motor
        self.direction_sign = direction_sign
        self.axis_index = axis_index

        if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC) != 0:
            raise RuntimeError(f"SDL_Init falló: {sdl2.SDL_GetError()}")

        n = sdl2.SDL_NumJoysticks()
        if n == 0:
            raise RuntimeError("No se detectó ningún joystick/volante conectado.")

        if joystick_index is None:
            print(f"      {n} dispositivo(s) detectado(s):")
            for i in range(n):
                name = sdl2.SDL_JoystickNameForIndex(i)
                name = name.decode() if name else "(sin nombre)"
                print(f"        [{i}] {name}")
            joystick_index = 0
            print(f"      Usando índice [0] por defecto — si no es el FFBeast, "
                  f"pasa joystick_index explícito.")

        self.joystick = sdl2.SDL_JoystickOpen(joystick_index)
        if not self.joystick:
            raise RuntimeError(f"No se pudo abrir el joystick {joystick_index}: {sdl2.SDL_GetError()}")

        if not sdl2.SDL_JoystickIsHaptic(self.joystick):
            raise RuntimeError(
                "El dispositivo seleccionado no reporta soporte de haptics. "
                "¿Es realmente el FFBeast? Prueba con otro joystick_index."
            )

        self.haptic = sdl2.SDL_HapticOpenFromJoystick(self.joystick)
        if not self.haptic:
            raise RuntimeError(f"SDL_HapticOpenFromJoystick falló: {sdl2.SDL_GetError()}")

        if sdl2.SDL_HapticRumbleSupported(self.haptic):
            pass  # solo informativo, no usamos rumble

        effect = sdl2.SDL_HapticEffect()
        ctypes.memset(ctypes.byref(effect), 0, ctypes.sizeof(effect))
        effect.type = sdl2.SDL_HAPTIC_CONSTANT
        effect.constant.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
        effect.constant.direction.dir[0] = 1
        effect.constant.length = sdl2.SDL_HAPTIC_INFINITY
        effect.constant.level = 0

        self._effect = effect
        self.effect_id = sdl2.SDL_HapticNewEffect(self.haptic, ctypes.byref(self._effect))
        if self.effect_id < 0:
            raise RuntimeError(f"SDL_HapticNewEffect falló: {sdl2.SDL_GetError()}")

        if sdl2.SDL_HapticRunEffect(self.haptic, self.effect_id, 1) != 0:
            raise RuntimeError(f"SDL_HapticRunEffect falló: {sdl2.SDL_GetError()}")

        self._last_theta = 0.0
        self._last_t = time.time()

    def _read_axis_rad(self):
        raw = sdl2.SDL_JoystickGetAxis(self.joystick, self.axis_index)  # -32768..32767
        norm = raw / 32768.0
        return norm * self.max_steering_rad

    def get_state(self):
        """
        Devuelve (theta_medido, theta_dot_diferencia_cruda).
        theta_medido es la única medición real y es lo que debe alimentar
        a SteeringKalmanFilter.update(). theta_dot_diferencia_cruda se deja
        solo para logging/comparación con el estimado del Kalman — el loop
        principal ya NO lo usa para alimentar al MPC.
        """
        theta = self._read_axis_rad()
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        theta_dot = (theta - self._last_theta) / dt
        self._last_theta = theta
        self._last_t = now
        return theta, theta_dot

    def apply_torque(self, tau):
        # Segunda capa de seguridad, independiente del tau_max del MPC:
        # aunque el MPC pidiera más, la planta nunca convierte a magnitud
        # más allá del límite físico real del motor.
        tau_clipped = float(np.clip(tau, -self.tau_max_motor, self.tau_max_motor))
        magnitude = int(np.clip(
            self.direction_sign * tau_clipped * self.nm_to_magnitude, -32767, 32767
        ))
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
            sdl2.SDL_QuitSubSystem(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC)
        except Exception:
            pass


def calibrate_ffbeast():
    """
    Modo de calibración manual: aplica pasos de torque pequeños y crecientes
    para que puedas ver/sentir a qué magnitud SDL2 responde el volante, y
    verificar el signo correcto de direction_sign ANTES de correr el MPC.
    Corre con: python mpc_monza_FINAL.py --calibrate
    """
    if not SDL2_AVAILABLE:
        print("Instala pysdl2 primero: pip install pysdl2 pysdl2-dll")
        return

    plant = FFBeastSteeringPlant(tau_max_motor=2.0, direction_sign=1)
    print("\nCalibración FFBeast — Ctrl+C para detener en cualquier momento")
    print("Mano cerca del corte de emergencia, no del volante.\n")
    input(">>> Enter para aplicar un torque de prueba pequeño y positivo <<<")

    try:
        for step_tau in [0.1, 0.2, 0.3]:
            print(f"\nAplicando tau = {step_tau} Nm (equiv.) por 1.5s...")
            t_end = time.time() + 1.5
            while time.time() < t_end:
                theta, theta_dot = plant.apply_torque(step_tau)
                print(f"  theta={math.degrees(theta):6.1f}°  "
                      f"theta_dot={math.degrees(theta_dot):7.1f}°/s", end="\r")
                time.sleep(0.05)
            plant.center()
            time.sleep(0.5)
            resp = input("¿Giró en la dirección correcta y de forma proporcional? [s/n] ")
            if resp.lower().startswith("n"):
                print("→ Ajusta direction_sign o nm_to_magnitude en el código y repite.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        plant.center()
        plant.close()
        print("\nCalibración terminada, volante liberado.")


# =============================================================================
# SECCIÓN 6 — ENVIAR A VJOY
# =============================================================================

class VJoyOutput:
    """Envía el ángulo del volante a vJoy → AC mueve el carro."""

    def __init__(self, device_id=1):
        self.j = pyvjoy.VJoyDevice(device_id)
        self.center()

    def send(self, steer_rad, max_rad=7.85):
        # max_rad unificado con MPCSteeringController.theta_max y
        # SimulatedSteeringPlant/ReferenceGenerator (450 grados totales)
        norm     = np.clip(steer_rad / max_rad, -1.0, 1.0)
        vjoy_val = int((norm + 1.0) / 2.0 * 32766 + 1)
        vjoy_val = max(1, min(32767, vjoy_val))
        self.j.data.wAxisX = vjoy_val
        self.j.update()

    def center(self):
        self.j.data.wAxisX = 16384
        self.j.update()


# =============================================================================
# SECCIÓN 7 — LOOP PRINCIPAL
# =============================================================================

def main(use_ffbeast=False):
    print("=" * 60)
    print("  MPC REAL (horizonte) — Monza | Alfa Romeo Giulietta")
    print("=" * 60)

    print("\n[1/5] Cargando trazada de Monza...")
    path = ReferencePath("monza_fast_lane.csv")
    print(f"      {path.n} puntos | {path.total_length:.0f} m de longitud ✓")

    print("[2/5] Inicializando Shared Memory de AC...")
    try:
        sm = ACSharedMemory()
        print("      Conexión establecida ✓")
    except Exception as e:
        print(f"      ERROR: {e}")
        print("      → Asegúrate de que Assetto Corsa está abierto con sesión activa")
        return

    print("[3/5] Inicializando vJoy...")
    try:
        vjoy = VJoyOutput(device_id=1)
        print("      vJoy Device 1 conectado ✓")
    except Exception as e:
        print(f"      ERROR: {e}")
        print("      → Instala vJoy y activa Device 1")
        return

    print("[4/5] Inicializando planta del volante...")
    if use_ffbeast:
        try:
            plant = FFBeastSteeringPlant()
            print("      FFBeast conectado, control por Constant Force (SDL2) ✓")
        except Exception as e:
            print(f"      ERROR: {e}")
            return
    else:
        plant = SimulatedSteeringPlant(Ts=0.05)
        print("      Planta simulada (Fase A, sin hardware) ✓")

    print("[5/5] Inicializando controlador MPC...")
    Ts = 0.05
    N = 10  # horizonte de 10 pasos * 0.05s = 0.5s
    ref_gen = ReferenceGenerator(wheelbase_L=2.7, steering_ratio_n=16, steer_sign=-1)
    mpc = MPCSteeringController(Ts=Ts, N=N)
    print(f"      N={N} pasos ({N*Ts:.2f}s horizonte), Q={mpc.Q}, R={mpc.R}, "
          f"τ_max=±{mpc.tau_max}Nm ✓")

    # Estimador de estado (Kalman): SOLO se usa en Fase B, donde theta viene
    # de una medición HID real y ruidosa. En Fase A el estado ya es exacto
    # (lo integra SimulatedSteeringPlant), así que no hace falta filtrarlo —
    # meterle Kalman ahí solo introduciría lag sin ganar nada.
    # Comparte A, B con el MPC (mismo modelo J,b,Ts) para que estimador y
    # controlador nunca discrepen sobre la dinámica asumida del volante.
    kf = None
    if use_ffbeast:
        kf = SteeringKalmanFilter(
            mpc.A, mpc.B,
            q_theta=1e-6,      # confianza alta en el modelo para theta
            q_theta_dot=1e-2,  # menos confianza en el modelo para theta_dot
            r_theta=1e-4,      # ajustar según el jitter real del eje HID
        )
        print("      Filtro de Kalman activo para estimar [theta, theta_dot] "
              "a partir de la medición real del eje (Fase B) ✓")
    last_tau_applied = 0.0

    log_name = f"mpc_session_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_name, 'w', newline='')
    writer   = csv.writer(log_file)
    writer.writerow([
        't_s', 'car_x', 'car_z',
        'e_y_m', 'e_psi_rad',
        'theta_ref0_deg', 'theta_real_deg',
        'theta_dot_degs', 'theta_dot_raw_degs',
        'error_deg', 'tau_Nm',
        'speed_kmh', 'kappa_avg', 'loop_dt_ms'
    ])

    print()
    print("INSTRUCCIONES:")
    print("  1. Entra a Monza en AC con la Giulietta")
    print("  2. Posiciona el carro en la recta principal")
    print("  3. Da gas suavemente (o usa la IA para velocidad)")
    print("  4. Presiona Enter para ACTIVAR el MPC")
    print("  5. Presiona Ctrl+C para detener")
    print()
    if use_ffbeast:
        print("ADVERTENCIA: el FFBeast va a mover el motor físico del volante.")
        print("             Ten a mano un corte de corriente físico, no solo Ctrl+C.")
    else:
        print("ADVERTENCIA: el MPC tomará control del volante virtual (vJoy).")
    print()
    input(">>> Presiona Enter para ACTIVAR el MPC <<<")

    print("\n¡MPC ACTIVO! Conduciendo en Monza...\n")

    last_packet_id  = None
    last_heading    = 0.0
    MIN_SPEED_MS    = 1.0
    cycle           = 0
    t0              = time.time()
    dt_warn_count   = 0

    print(f"{'t(s)':>6} | {'e_y(m)':>7} | {'e_ψ(°)':>7} | "
          f"{'θ_ref0(°)':>10} | {'θ_real(°)':>10} | "
          f"{'τ(Nm)':>7} | {'v(km/h)':>8} | {'dt(ms)':>7}")
    print("-" * 85)

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
            speed_ms  = ph.speedKmh / 3.6
            speed_kmh = ph.speedKmh

            car_vx, _, car_vz = ph.velocity
            if speed_ms > MIN_SPEED_MS:
                car_heading  = math.atan2(car_vz, car_vx)
                last_heading = car_heading
            else:
                car_heading = last_heading

            e_y, e_psi, s, idx = path.frenet(car_x, car_z, car_heading)
            kappa_avg = path.curvature_at_offset(idx, 20.0)

            # --- Estado que alimenta al MPC ---
            # Fase A: estado exacto de la planta simulada, sin filtrar.
            # Fase B: el Kalman predice con el último torque aplicado y
            # corrige con la medición real de theta de este ciclo; el
            # theta_dot que sale de aquí YA NO es la diferencia numérica
            # cruda de antes, sino el estimado del filtro.
            theta_dot_raw_for_log = 0.0
            if use_ffbeast:
                theta_meas, theta_dot_raw_for_log = plant.get_state()
                kf.predict(last_tau_applied)
                kf.update(theta_meas)
                theta_current, theta_dot_current = kf.state()
            else:
                theta_current, theta_dot_current = plant.get_state()

            if speed_ms > MIN_SPEED_MS:
                theta_ref_seq = ref_gen.generate(
                    e_y, e_psi, path, idx, speed_ms, N, Ts
                )
                tau, theta_pred, theta_dot_pred = mpc.solve(
                    theta_current, theta_dot_current, theta_ref_seq
                )
                theta_new, _ = plant.apply_torque(tau)
            else:
                tau = 0.0
                plant.center()
                theta_new, _ = plant.get_state()
                theta_ref_seq = np.zeros(N)
                # Reinicia el estado interno del MPC (u_prev, warm-start) al
                # detenerse, en Fase A y en Fase B, para no arrastrar valores
                # de antes de la pausa cuando el control se reengancha.
                mpc.reset()
                if use_ffbeast:
                    kf.reset(theta_new, 0.0)

            last_tau_applied = tau

            vjoy.send(theta_new)

            cycle += 1
            t_elapsed = time.time() - t0
            loop_dt_ms = (time.time() - t_loop) * 1000.0

            if cycle % 10 == 0:
                print(f"{t_elapsed:>6.1f} | "
                      f"{e_y:>7.2f} | "
                      f"{math.degrees(e_psi):>7.1f} | "
                      f"{math.degrees(theta_ref_seq[0]):>10.1f} | "
                      f"{math.degrees(theta_new):>10.1f} | "
                      f"{tau:>7.3f} | "
                      f"{speed_kmh:>8.1f} | "
                      f"{loop_dt_ms:>7.1f}")

            writer.writerow([
                round(t_elapsed, 3),
                round(car_x, 3), round(car_z, 3),
                round(e_y, 4), round(e_psi, 5),
                round(math.degrees(theta_ref_seq[0]), 3),
                round(math.degrees(theta_new), 3),
                round(math.degrees(theta_dot_current), 3),
                round(math.degrees(theta_dot_raw_for_log), 3),
                round(math.degrees(theta_ref_seq[0] - theta_new), 3),
                round(tau, 4),
                round(speed_kmh, 2),
                round(kappa_avg, 6),
                round(loop_dt_ms, 2),
            ])

            if cycle % 20 == 0:
                log_file.flush()

            # Mantener el periodo de muestreo — avisar si el loop no llega a tiempo,
            # porque un Ts real distinto al Ts asumido por el modelo del MPC
            # degrada la calidad del control (más notorio en Fase B con hardware real)
            elapsed = time.time() - t_loop
            sleep_t = Ts - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
            else:
                dt_warn_count += 1
                if dt_warn_count % 20 == 1:
                    print(f"      ⚠ loop tardó {elapsed*1000:.1f}ms (> Ts={Ts*1000:.0f}ms), "
                          f"el control se está retrasando")

    except KeyboardInterrupt:
        print("\n\nMPC detenido por el usuario")

    finally:
        plant.center()
        vjoy.center()
        plant.close()
        log_file.flush()
        log_file.close()
        sm.close()
        print(f"Volante centrado ✓")
        print(f"Log guardado: {log_name}")
        print(f"Ciclos ejecutados: {cycle}")
        print(f"Tiempo total: {time.time()-t0:.1f}s")


if __name__ == "__main__":
    import sys
    # python mpc_monza_FINAL.py               → Fase A, planta simulada + vJoy
    # python mpc_monza_FINAL.py --ffbeast      → Fase B, motor real (FFBeast) + vJoy
    # python mpc_monza_FINAL.py --calibrate    → calibración manual del FFBeast
    if "--calibrate" in sys.argv:
        calibrate_ffbeast()
    else:
        use_ffbeast = "--ffbeast" in sys.argv
        main(use_ffbeast=use_ffbeast)