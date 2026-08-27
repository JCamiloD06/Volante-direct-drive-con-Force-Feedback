"""
mpc_localization.py

Corre esto en la PC donde tenes Assetto Corsa abierto (con la sesion ya
cargada en pista). Lee la posicion del auto en tiempo real via Shared
Memory, la compara contra la trayectoria de referencia de Monza
(monza_fast_lane.csv) y calcula el estado que necesita el MPC:

    e_y      -> error lateral (cross-track error), en metros
    e_psi    -> error de orientacion (heading error), en radianes
    s        -> distancia recorrida a lo largo de la pista, en metros
    v        -> velocidad del auto, en m/s
    steer    -> angulo de las ruedas (steerAngle de la fisica de AC)
    gas/brake -> inputs del pedal, 0 a 1
    curvature_horizon -> curvatura de los proximos N metros de pista

Ademas de imprimir estos valores en vivo, cada corrida genera un CSV
("session_log_<fecha>_<hora>.csv") con todo el historial de la sesion,
incluyendo tambien la posicion cruda del auto (car_x, car_y_elevation,
car_z, en el sistema de coordenadas mundo de AC) para poder graficar
despues el trazado REAL manejado, no solo la aproximacion sobre la
linea de referencia.

Requisitos:
    - Windows (la Shared Memory de AC solo existe en Windows)
    - Assetto Corsa corriendo, con una sesion activa en Monza
    - pip install numpy

No requiere paquetes de terceros para leer la memoria compartida: usa
ctypes + mmap directo sobre los named shared memory objects que expone
el juego (acpmf_physics, acpmf_graphics), documentados publicamente en
el SDK oficial de apps de Assetto Corsa.
"""

import ctypes
import mmap
import math
import csv
import time
import numpy as np


# ---------------------------------------------------------------------
# 1. Estructuras de Shared Memory (SDK oficial de Assetto Corsa)
# ---------------------------------------------------------------------

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
    """Wrapper minimo para leer los bloques de shared memory de AC."""

    def __init__(self):
        self._physics_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysics), "acpmf_physics")
        self._graphics_mmap = mmap.mmap(-1, ctypes.sizeof(SPageFileGraphic), "acpmf_graphics")

    def read_physics(self) -> SPageFilePhysics:
        return SPageFilePhysics.from_buffer_copy(self._physics_mmap)

    def read_graphics(self) -> SPageFileGraphic:
        return SPageFileGraphic.from_buffer_copy(self._graphics_mmap)

    def close(self):
        self._physics_mmap.close()
        self._graphics_mmap.close()


# ---------------------------------------------------------------------
# 2. Trayectoria de referencia (CSV extraido de fast_lane.ai)
# ---------------------------------------------------------------------

class ReferencePath:
    """
    Carga el centerline de Monza y precalcula heading/curvatura en cada
    punto. Provee proyeccion de una posicion (x,z) arbitraria sobre la
    trayectoria para obtener el error lateral y de orientacion (Frenet).
    """

    def __init__(self, csv_path: str):
        idx, xs, zs, s = [], [], [], []
        with open(csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                idx.append(int(row["index"]))
                xs.append(float(row["x"]))
                zs.append(float(row["z"]))
                s.append(float(row["cumulative_length_m"]))

        self.x = np.array(xs)
        self.z = np.array(zs)
        self.s = np.array(s)
        self.n = len(self.x)
        self.total_length = self.s[-1]

        # Heading (tangente) en cada punto: angulo del segmento hacia el
        # siguiente punto, en el plano x-z. atan2(dz, dx) por convencion.
        dx = np.roll(self.x, -1) - self.x
        dz = np.roll(self.z, -1) - self.z
        self.heading = np.arctan2(dz, dx)

        # Curvatura: derivada del heading respecto al arco (dtheta/ds).
        # Se usa diferencia angular "wrapeada" a [-pi, pi] para evitar
        # saltos falsos al cruzar +-pi. ds se calcula como distancia
        # euclidiana real entre puntos consecutivos (incluyendo el
        # segmento de cierre ultimo->primero), no como resta de
        # longitudes acumuladas, para evitar un pico falso de curvatura
        # justo en el punto de cierre de vuelta.
        dtheta = np.roll(self.heading, -1) - self.heading
        dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
        next_x = np.roll(self.x, -1)
        next_z = np.roll(self.z, -1)
        ds = np.sqrt((next_x - self.x) ** 2 + (next_z - self.z) ** 2)
        ds[ds < 1e-3] = 1e-3
        self.curvature = dtheta / ds

        self._last_idx = 0  # cache para busqueda local rapida

    def _nearest_index(self, car_x: float, car_z: float, search_window: int = 60) -> int:
        """
        Busca el punto mas cercano de la trayectoria. Como el auto se
        mueve de forma continua, primero busca en una ventana alrededor
        del ultimo indice encontrado (barato). Si no encuentra nada
        razonablemente cerca ahi (p.ej. teletransporte / primer frame),
        hace busqueda global una sola vez.
        """
        idxs = (self._last_idx + np.arange(-search_window, search_window)) % self.n
        d2 = (self.x[idxs] - car_x) ** 2 + (self.z[idxs] - car_z) ** 2
        best_local = idxs[np.argmin(d2)]

        if d2.min() > 400:  # > 20 m del ultimo punto conocido: probable salto
            d2_global = (self.x - car_x) ** 2 + (self.z - car_z) ** 2
            best_local = int(np.argmin(d2_global))

        self._last_idx = int(best_local)
        return self._last_idx

    def compute_frenet_error(self, car_x: float, car_z: float, car_heading: float):
        """
        Devuelve (e_y, e_psi, s, idx) para la posicion/orientacion dadas.

        e_y   > 0 si el auto esta a la izquierda de la trayectoria de referencia
        e_psi = heading_auto - heading_pista, wrapeado a [-pi, pi]
        """
        idx = self._nearest_index(car_x, car_z)

        # Vector auto -> punto de referencia mas cercano
        rx, rz = self.x[idx], self.z[idx]
        path_heading = self.heading[idx]

        # Proyeccion sobre la normal de la trayectoria (signo = lado)
        dx = car_x - rx
        dz = car_z - rz
        # Normal a la tangente (tangente = (cos h, sin h)) -> normal = (-sin h, cos h)
        e_y = -math.sin(path_heading) * dx + math.cos(path_heading) * dz

        e_psi = car_heading - path_heading
        e_psi = (e_psi + math.pi) % (2 * math.pi) - math.pi

        return e_y, e_psi, self.s[idx], idx

    def curvature_horizon(self, idx: int, horizon_m: float = 80.0, step_m: float = 2.0):
        """
        Curvatura de la pista en los proximos `horizon_m` metros desde el
        indice actual, muestreada cada `step_m`. Sirve como referencia
        futura para el horizonte de prediccion del MPC.
        """
        n_steps = int(horizon_m / step_m)
        out = []
        # avance aproximado: como los puntos estan a ~1.53 m entre si,
        # convertimos metros a "pasos de indice" aproximados
        avg_spacing = self.total_length / self.n
        idx_step = max(1, round(step_m / avg_spacing))
        for k in range(n_steps):
            i = (idx + k * idx_step) % self.n
            out.append(self.curvature[i])
        return np.array(out)


# ---------------------------------------------------------------------
# 3. Loop principal
# ---------------------------------------------------------------------

def main():
    path = ReferencePath("monza_fast_lane.csv")
    sm = ACSharedMemory()

    log_filename = f"session_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_filename, "w", newline="")
    log_writer = csv.writer(log_file)
    log_writer.writerow([
        "t_s", "car_x", "car_y_elevation", "car_z",
        "e_y_m", "e_psi_rad", "s_m", "v_ms",
        "steer_angle_rad", "steer_angle_deg", "gas", "brake",
        "kappa_next_10m_avg",
    ])

    print(f"Trayectoria cargada: {path.n} puntos, {path.total_length:.1f} m de longitud")
    print(f"Guardando log en: {log_filename}")
    print("Esperando datos del juego (Ctrl+C para salir y cerrar el log)...\n")

    # IMPORTANTE: no usamos ph.heading directo. El heading interno de AC
    # usa una convencion de ejes propia (se detecto un offset sistematico
    # de ~90 grados contra el heading calculado a partir del CSV). En vez
    # de adivinar esa convencion, calculamos el heading del auto con el
    # mismo criterio que el heading de la pista: atan2 sobre las
    # componentes x,z de un vector de "hacia donde se mueve" -- en este
    # caso el vector velocidad, que esta en el mismo sistema de
    # coordenadas mundo que carCoordinates. Asi ambos angulos quedan
    # garantizados en la misma referencia.
    last_heading = 0.0
    MIN_SPEED_FOR_HEADING = 2.0  # m/s; por debajo de esto atan2(vz,vx) es ruidoso

    last_packet_id = None
    n_logged = 0
    t0 = time.time()

    try:
        while True:
            gr = sm.read_graphics()
            ph = sm.read_physics()

            # Si el packetId no cambio, es el mismo frame que ya leimos
            # (juego pausado, en menu, o Alt-tab) -> no lo logueamos para
            # no llenar el archivo de filas duplicadas.
            if gr.packetId == last_packet_id:
                time.sleep(0.01)
                continue
            last_packet_id = gr.packetId

            car_x, car_y, car_z = gr.carCoordinates
            speed_ms = ph.speedKmh / 3.6

            car_vx, _car_vy, car_vz = ph.velocity
            if speed_ms > MIN_SPEED_FOR_HEADING:
                car_heading = math.atan2(car_vz, car_vx)
                last_heading = car_heading
            else:
                car_heading = last_heading  # mantiene el ultimo heading valido a baja velocidad

            e_y, e_psi, s, idx = path.compute_frenet_error(car_x, car_z, car_heading)
            kappa_horizon = path.curvature_horizon(idx)
            kappa_avg = float(np.mean(kappa_horizon[:5]))

            steer_rad = ph.steerAngle
            steer_deg = math.degrees(steer_rad)
            t = time.time() - t0

            state = {
                "car_x": round(car_x, 2),
                "car_z": round(car_z, 2),
                "e_y_m": round(e_y, 3),
                "e_psi_rad": round(e_psi, 4),
                "s_m": round(s, 2),
                "v_ms": round(speed_ms, 2),
                "steer_deg": round(steer_deg, 2),
                "gas": round(ph.gas, 2),
                "brake": round(ph.brake, 2),
                "kappa_next_10m_avg": round(kappa_avg, 5),
            }
            print(state)

            log_writer.writerow([
                round(t, 3), round(car_x, 4), round(car_y, 4), round(car_z, 4),
                round(e_y, 4), round(e_psi, 5), round(s, 3),
                round(speed_ms, 3), round(steer_rad, 5), round(steer_deg, 3),
                round(ph.gas, 3), round(ph.brake, 3), round(kappa_avg, 6),
            ])
            n_logged += 1
            if n_logged % 20 == 0:
                log_file.flush()  # asegura que quede guardado en disco periodicamente

            time.sleep(0.05)  # ~20 Hz, ajustar segun frecuencia que necesite el MPC

    except KeyboardInterrupt:
        pass
    finally:
        log_file.flush()
        log_file.close()
        sm.close()
        print(f"\nLog cerrado: {log_filename} ({n_logged} filas guardadas)")


if __name__ == "__main__":
    main()
