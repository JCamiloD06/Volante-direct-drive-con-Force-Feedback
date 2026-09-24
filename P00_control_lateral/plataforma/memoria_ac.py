"""
Lectura de la memoria compartida de Assetto Corsa y estado del vehículo.

Copiado y ampliado desde mpc_monza_Completo_barrido.py, clases
SPageFilePhysics, SPageFileGraphic, SPageFileStaticHead y ACSharedMemory.

Ampliación de la página de física. Después del campo abs se declaran los
campos hasta localVelocity según la librería comunitaria
mdjarv/assettocorsasharedmemory, archivo Physics.cs. Esa parte NO ESTÁ
VERIFICADA en Assetto Corsa 1.16.4 y se valida en la fase 3. Mientras tanto,
cada lectura comprueba que los puntos de contacto sean coherentes antes de
usarlos y, si no lo son, se usa el traslado de respaldo de la decisión I2.

Orden de ruedas supuesto en los arreglos, delantera izquierda, delantera
derecha, trasera izquierda, trasera derecha. NO VERIFICADO. Un intercambio
entre izquierda y derecha no altera los puntos medios de eje. Un intercambio
entre ejes lo detecta la comprobación de alineación con ψ.
"""
import ctypes
import math
import mmap
from dataclasses import dataclass

from plataforma.angulos import envolver

_F = ctypes.c_float
_I = ctypes.c_int32


class Coordenadas(ctypes.Structure):
    _pack_ = 4
    _fields_ = [("x", _F), ("y", _F), ("z", _F)]


_CAMPOS_FISICA_BASE = [
    ("packetId", _I),
    ("gas", _F),
    ("brake", _F),
    ("fuel", _F),
    ("gear", _I),
    ("rpms", _I),
    ("steerAngle", _F),
    ("speedKmh", _F),
    ("velocity", _F * 3),
    ("accG", _F * 3),
    ("wheelSlip", _F * 4),
    ("wheelLoad", _F * 4),
    ("wheelsPressure", _F * 4),
    ("wheelAngularSpeed", _F * 4),
    ("tyreWear", _F * 4),
    ("tyreDirtyLevel", _F * 4),
    ("tyreCoreTemperature", _F * 4),
    ("camberRAD", _F * 4),
    ("suspensionTravel", _F * 4),
    ("drs", _F),
    ("tc", _F),
    ("heading", _F),
    ("pitch", _F),
    ("roll", _F),
    ("cgHeight", _F),
    ("carDamage", _F * 5),
    ("numberOfTyresOut", _I),
    ("pitLimiterOn", _I),
    ("abs", _F),
]

# Campos posteriores a abs. Fuente comunitaria, NO VERIFICADOS.
_CAMPOS_FISICA_AMPLIADOS = [
    ("kersCharge", _F),
    ("kersInput", _F),
    ("autoShifterOn", _I),
    ("rideHeight", _F * 2),
    ("turboBoost", _F),
    ("ballast", _F),
    ("airDensity", _F),
    ("airTemp", _F),
    ("roadTemp", _F),
    ("localAngularVel", _F * 3),
    ("finalFF", _F),
    ("performanceMeter", _F),
    ("engineBrake", _I),
    ("ersRecoveryLevel", _I),
    ("ersPowerLevel", _I),
    ("ersHeatCharging", _I),
    ("ersIsCharging", _I),
    ("kersCurrentKJ", _F),
    ("drsAvailable", _I),
    ("drsEnabled", _I),
    ("brakeTemp", _F * 4),
    ("clutch", _F),
    ("tyreTempI", _F * 4),
    ("tyreTempM", _F * 4),
    ("tyreTempO", _F * 4),
    ("isAIControlled", _I),
    ("tyreContactPoint", Coordenadas * 4),
    ("tyreContactNormal", Coordenadas * 4),
    ("tyreContactHeading", Coordenadas * 4),
    ("brakeBias", _F),
    ("localVelocity", _F * 3),
]


class SPageFilePhysics(ctypes.Structure):
    _pack_ = 4
    _fields_ = _CAMPOS_FISICA_BASE


class SPageFilePhysicsAmpliada(ctypes.Structure):
    _pack_ = 4
    _fields_ = _CAMPOS_FISICA_BASE + _CAMPOS_FISICA_AMPLIADOS


class SPageFileGraphic(ctypes.Structure):
    _pack_ = 4
    _fields_ = [
        ("packetId", _I),
        ("status", _I),
        ("session", _I),
        ("currentTime", ctypes.c_wchar * 15),
        ("lastTime", ctypes.c_wchar * 15),
        ("bestTime", ctypes.c_wchar * 15),
        ("split", ctypes.c_wchar * 15),
        ("completedLaps", _I),
        ("position", _I),
        ("iCurrentTime", _I),
        ("iLastTime", _I),
        ("iBestTime", _I),
        ("sessionTimeLeft", _F),
        ("distanceTraveled", _F),
        ("isInPit", _I),
        ("currentSectorIndex", _I),
        ("lastSectorTime", _I),
        ("numberOfLaps", _I),
        ("tyreCompound", ctypes.c_wchar * 33),
        ("replayTimeMultiplier", _F),
        ("normalizedCarPosition", _F),
        ("carCoordinates", _F * 3),
    ]


class SPageFileStaticHead(ctypes.Structure):
    """Solo el encabezado de la página estática. Esta página no lleva packetId."""
    _pack_ = 4
    _fields_ = [
        ("smVersion", ctypes.c_wchar * 15),
        ("acVersion", ctypes.c_wchar * 15),
        ("numberOfSessions", _I),
        ("numCars", _I),
        ("carModel", ctypes.c_wchar * 33),
        ("track", ctypes.c_wchar * 33),
    ]


class MemoriaAC:
    """Abre las tres páginas. Intenta primero la página de física ampliada."""

    def __init__(self, usar_fisica_ampliada=True):
        self.fisica_ampliada = False
        self.motivo_sin_ampliada = None
        self._ph = None
        if usar_fisica_ampliada:
            try:
                self._ph = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysicsAmpliada), "acpmf_physics")
                self._tipo_fisica = SPageFilePhysicsAmpliada
                self.fisica_ampliada = True
            except Exception as e:
                self.motivo_sin_ampliada = f"{type(e).__name__}: {e}"
        if self._ph is None:
            self._ph = mmap.mmap(-1, ctypes.sizeof(SPageFilePhysics), "acpmf_physics")
            self._tipo_fisica = SPageFilePhysics
        self._gr = mmap.mmap(-1, ctypes.sizeof(SPageFileGraphic), "acpmf_graphics")
        try:
            self._st = mmap.mmap(-1, ctypes.sizeof(SPageFileStaticHead), "acpmf_static")
        except Exception:
            self._st = None

    def leer_fisica(self):
        return self._tipo_fisica.from_buffer_copy(self._ph)

    def leer_graficos(self):
        return SPageFileGraphic.from_buffer_copy(self._gr)

    def leer_estatica(self):
        """Devuelve vehículo, pista y versión, o None en cada campo ilegible."""
        info = {"carModel": None, "track": None, "acVersion": None, "lectura": "no disponible"}
        if self._st is None:
            return info

        def limpio(valor):
            texto = str(valor).split("\x00")[0].strip()
            if not texto or not all(32 <= ord(c) < 127 for c in texto):
                return None
            return texto

        try:
            st = SPageFileStaticHead.from_buffer_copy(self._st)
            info["carModel"] = limpio(st.carModel)
            info["track"] = limpio(st.track)
            info["acVersion"] = limpio(st.acVersion)
            leidos = sum(1 for k in ("carModel", "track", "acVersion") if info[k])
            info["lectura"] = "ok" if leidos == 3 else f"parcial ({leidos} de 3)"
        except Exception as e:
            info["lectura"] = f"error: {e}"
        return info

    def cerrar(self):
        self._ph.close()
        self._gr.close()
        if self._st is not None:
            self._st.close()


@dataclass
class EstadoVehiculo:
    t_lectura: float
    packet_graficos: int
    packet_fisica: int
    x: float
    y: float
    z: float
    heading_ac: float
    psi: float
    rumbo_velocidad: float
    v_ms: float
    v_kmh: float
    steer_angle_ac: float
    acc_g_x: float
    acc_g_y: float
    acc_g_z: float
    gas_ac: float
    freno_ac: float
    ruedas_fuera: int
    vueltas_completadas: int
    posicion_normalizada: float
    en_pits: int
    contactos_ok: bool
    motivo_contactos: str
    x_tras: float
    z_tras: float
    x_del: float
    z_del: float
    L_medida: float
    L_usada: float
    desalineacion_ejes: float
    tasa_guinada_local: float


def construir_estado(gr, ph, cfg_vehiculo, t_lectura, fisica_ampliada):
    """
    Arma el estado del vehículo a partir de una lectura de gráficos y física.

    ψ es la orientación de la carrocería, decisión I1. Los ejes salen de los
    puntos de contacto cuando pasan las comprobaciones, decisión I2. Si no,
    se trasladan desde la posición de Assetto Corsa con los valores de
    respaldo de la configuración y se registra el motivo.
    """
    x, y, z = (float(c) for c in gr.carCoordinates)
    heading = float(ph.heading)
    psi = envolver(heading + float(cfg_vehiculo["desfase_heading_rad"]))
    v_kmh = float(ph.speedKmh)
    v_ms = v_kmh / 3.6
    vx, _, vz = (float(c) for c in ph.velocity)
    rumbo = math.atan2(vz, vx) if v_ms > 1.0 else float("nan")

    contactos_ok = False
    motivo = "fisica ampliada no disponible"
    L_medida = float("nan")
    desalineacion = float("nan")
    tasa_guinada = float("nan")
    x_tras = z_tras = x_del = z_del = float("nan")

    if fisica_ampliada:
        tasa_guinada = float(ph.localAngularVel[1])
        pts = [(float(c.x), float(c.z)) for c in ph.tyreContactPoint]
        i_del = cfg_vehiculo["indices_delanteras"]
        i_tras = cfg_vehiculo["indices_traseras"]
        xd = 0.5 * (pts[i_del[0]][0] + pts[i_del[1]][0])
        zd = 0.5 * (pts[i_del[0]][1] + pts[i_del[1]][1])
        xt = 0.5 * (pts[i_tras[0]][0] + pts[i_tras[1]][0])
        zt = 0.5 * (pts[i_tras[0]][1] + pts[i_tras[1]][1])
        L = math.hypot(xd - xt, zd - zt)
        centro = math.hypot(0.5 * (xd + xt) - x, 0.5 * (zd + zt) - z)
        todos = [c for p in pts for c in p]
        if not all(math.isfinite(c) for c in todos) or all(c == 0.0 for c in todos):
            motivo = "contactos nulos o no finitos"
        elif not (cfg_vehiculo["L_min_m"] <= L <= cfg_vehiculo["L_max_m"]):
            motivo = f"batalla fuera de rango {L:.3f} m"
        elif centro > cfg_vehiculo["distancia_max_centro_m"]:
            motivo = f"ejes lejos de la posicion {centro:.2f} m"
        else:
            desalineacion = envolver(math.atan2(zd - zt, xd - xt) - psi)
            if abs(desalineacion) > cfg_vehiculo["desalineacion_max_rad"]:
                motivo = f"ejes desalineados con psi {math.degrees(desalineacion):.1f} grados"
            else:
                contactos_ok = True
                motivo = "ok"
                L_medida = L
                x_tras, z_tras, x_del, z_del = xt, zt, xd, zd

    if contactos_ok and not cfg_vehiculo.get("forzar_respaldo", False):
        L_usada = L_medida
    else:
        if contactos_ok:
            motivo = "respaldo forzado por configuracion"
            contactos_ok = False
        resp = cfg_vehiculo["respaldo"]
        L_usada = float(resp["L_m"])
        a = float(resp["fraccion_punto_desde_trasero"]) * L_usada
        c, s = math.cos(psi), math.sin(psi)
        x_tras, z_tras = x - a * c, z - a * s
        x_del, z_del = x + (L_usada - a) * c, z + (L_usada - a) * s

    return EstadoVehiculo(
        t_lectura=t_lectura,
        packet_graficos=int(gr.packetId),
        packet_fisica=int(ph.packetId),
        x=x, y=y, z=z,
        heading_ac=heading,
        psi=psi,
        rumbo_velocidad=rumbo,
        v_ms=v_ms,
        v_kmh=v_kmh,
        steer_angle_ac=float(ph.steerAngle),
        acc_g_x=float(ph.accG[0]),
        acc_g_y=float(ph.accG[1]),
        acc_g_z=float(ph.accG[2]),
        gas_ac=float(ph.gas),
        freno_ac=float(ph.brake),
        ruedas_fuera=int(ph.numberOfTyresOut),
        vueltas_completadas=int(gr.completedLaps),
        posicion_normalizada=float(gr.normalizedCarPosition),
        en_pits=int(gr.isInPit),
        contactos_ok=contactos_ok,
        motivo_contactos=motivo,
        x_tras=x_tras, z_tras=z_tras, x_del=x_del, z_del=z_del,
        L_medida=L_medida,
        L_usada=L_usada,
        desalineacion_ejes=desalineacion,
        tasa_guinada_local=tasa_guinada,
    )
