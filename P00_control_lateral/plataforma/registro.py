"""
Registro por corrida.

Acumula las filas en memoria y las escribe al terminar, también si la corrida
se interrumpe, para no poner escritura a disco dentro del ciclo de control.
Cada corrida produce una carpeta con telemetria.csv, perfil.csv y
manifiesto.json con configuración, versiones y huellas.
"""
import csv
import json
import os
import platform
import time

from plataforma import procedencia

COLUMNAS = [
    # Tiempo y bucle
    "ciclo", "t_s", "periodo_ms", "tc_ms", "retardo_ms",
    "packet_graficos", "packet_fisica", "packets_repetidos",
    # Posición y orientación
    "x", "y", "z", "psi_rad", "heading_ac_rad", "rumbo_velocidad_rad", "deriva_rad",
    # Ejes
    "contactos_ok", "motivo_contactos", "x_tras", "z_tras", "x_del", "z_del",
    "L_medida_m", "L_usada_m", "desalineacion_ejes_rad",
    # Proyecciones
    "idx_cg", "s_cg_m", "e_y_cg_m", "e_psi_cg_rad", "kappa_cg",
    "idx_tras", "s_tras_m", "e_y_tras_m", "e_psi_tras_rad", "kappa_tras",
    "idx_del", "s_del_m", "e_y_del_m", "e_psi_del_rad",
    # Velocidad
    "v_kmh", "v_ref_kmh", "v_perfil_kmh",
    # Dirección
    "delta_pedido_rad", "delta_aplicado_rad", "factor_direccion", "direccion_retenida",
    "sat_magnitud", "sat_tasa",
    "steer_norm", "sat_eje", "steer_angle_ac",
    # Controlador lateral
    "ctrl_estado", "ctrl_iteraciones", "ctrl_respaldo", "ctrl_extra1", "ctrl_extra2",
    # Longitudinal
    "u_pid", "u_pedal", "gas_cmd", "freno_cmd", "gas_ac", "freno_ac",
    # Vehículo y vuelta
    "acc_g_x", "acc_g_y", "acc_g_z", "tasa_guinada_local",
    "ruedas_fuera", "vueltas_completadas", "posicion_normalizada", "en_pits", "fase_vuelta",
]


def _versiones():
    v = {"python": platform.python_version(), "plataforma": platform.platform()}
    for nombre in ("numpy", "scipy", "osqp", "pyvjoy"):
        try:
            modulo = __import__(nombre)
            v[nombre] = getattr(modulo, "__version__", "sin version")
        except Exception:
            v[nombre] = None
    return v


class RegistroCorrida:
    def __init__(self, directorio_base, id_corrida, config, ruta_config, meta):
        self.dir = os.path.join(directorio_base, id_corrida)
        os.makedirs(self.dir, exist_ok=True)
        self.id = id_corrida
        self.config = config
        self.ruta_config = ruta_config
        self.meta = dict(meta)
        self.filas = []
        self.notas = []
        self.t_inicio = time.strftime("%Y-%m-%dT%H:%M:%S")

    def agregar(self, fila):
        self.filas.append(fila)

    def nota(self, texto):
        self.notas.append(texto)

    def _escribir_telemetria(self):
        ruta = os.path.join(self.dir, "telemetria.csv")
        with open(ruta, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=COLUMNAS, extrasaction="ignore")
            w.writeheader()
            w.writerows(self.filas)

    def _escribir_perfil(self, trazada, perfil):
        ruta = os.path.join(self.dir, "perfil.csv")
        with open(ruta, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["idx", "s_m", "x", "z", "heading_rad", "curvatura_suavizada",
                        "v_limite_kmh", "v_perfil_kmh"])
            for i in range(trazada.n):
                w.writerow([i, round(float(trazada.s[i]), 3), round(float(trazada.x[i]), 3),
                            round(float(trazada.z[i]), 3), round(float(trazada.heading[i]), 6),
                            float(trazada.curvature[i]), round(float(perfil.v_limit[i]) * 3.6, 3),
                            round(float(perfil.v_max[i]) * 3.6, 3)])

    def escribir(self, trazada, perfil, resumen, info_estatica):
        self._escribir_telemetria()
        self._escribir_perfil(trazada, perfil)
        manifiesto = {
            "id_corrida": self.id,
            "inicio": self.t_inicio,
            "fin": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "ciclos": len(self.filas),
            "meta": self.meta,
            "config": self.config,
            "ruta_config": str(self.ruta_config),
            "sha256_config": procedencia.sha256(self.ruta_config),
            "versiones": _versiones(),
            "origen": procedencia.estado_origen(),
            "sha256_codigo_p00": procedencia.huellas_codigo_p00(),
            "assetto_corsa": info_estatica,
            "resumen": resumen,
            "notas": self.notas,
        }
        with open(os.path.join(self.dir, "manifiesto.json"), "w", encoding="utf-8") as f:
            json.dump(manifiesto, f, indent=2, ensure_ascii=False, default=str)
        return self.dir
