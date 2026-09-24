"""
Planta longitudinal identificada con telemetría de vuelta y validada en lazo cerrado.

El modelo G(s) = 248.7/(17.2s+1) se identificó solo con acelerador a fondo y
no representa el freno ni el acelerador parcial. En las vueltas de verificación
subestima la aceleración con gas en unos 2.2 km/h/s y la deceleración con freno
en unos 2.6 km/h/s. Por eso el PID ajustado sobre él falló en pista el 2026-09-16.

Modelo por tramos, con v en km/h y el pedal u entre menos uno y uno aplicado con
un ciclo de retardo.
    u >= 0   dv/dt = a_g u - c_g v + d_g
    u <  0   dv/dt = a_f u - c_f v + d_f
Se ajusta por mínimos cuadrados en una vuelta de entrenamiento y se valida
simulando el lazo cerrado completo, perfil con anticipación, PID y conformador
de pedal de plataforma/longitudinal.py, contra vueltas no usadas en el ajuste,
una con cada juego de ganancias.

Uso
    python analisis/planta_longitudinal.py
Escribe en sintonia_pid/ la planta, el perfil para Simulink y la validación.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np
from scipy.io import savemat

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))
from plataforma.longitudinal import PIDLongitudinalController, PedalShaper  # noqa: E402

CORRIDAS = RAIZ.parent / "data" / "raw" / "p00" / "corridas"
SALIDA = RAIZ / "sintonia_pid"
TS = 0.05
RETARDO_CICLOS = 1
PREVIEW_M, PREVIEW_GANANCIA = 20.0, 0.35

ENTRENAMIENTO = "verificacion_s00_stanley_conservador_20260915_223523"
VALIDACION = {
    "verificacion_s00_stanley_conservador_20260916_185552": "base.json",
    "verificacion_s00_stanley_conservador_20260916_205515": "prueba_pid_simulink.json",
}
COLUMNAS = ["t_s", "v_kmh", "v_ref_kmh", "v_perfil_kmh", "u_pedal", "s_cg_m"]


def cargar(corrida):
    with open(CORRIDAS / corrida / "telemetria.csv", encoding="utf-8") as f:
        a = np.array([[r[c] for c in COLUMNAS] for r in csv.DictReader(f)], dtype=float)
    return {c: a[:, i] for i, c in enumerate(COLUMNAS)}


def ajustar(corrida):
    T = cargar(corrida)
    v, u = T["v_kmh"], T["u_pedal"]
    dv = np.gradient(v, T["t_s"])
    n = len(v) - RETARDO_CICLOS
    u0, v1, a1 = u[:n], v[RETARDO_CICLOS:], dv[RETARDO_CICLOS:]
    valido = (v1 > 20) & (np.abs(a1) < 80)
    X = np.c_[u0, -v1, np.ones(n)]
    planta = {}
    for nombre, tramo in (("gas", valido & (u0 >= 0)), ("freno", valido & (u0 < 0))):
        c, *_ = np.linalg.lstsq(X[tramo], a1[tramo], rcond=None)
        r = a1[tramo] - X[tramo] @ c
        planta[nombre] = {"a": float(c[0]), "c": float(c[1]), "d": float(c[2]),
                          "rmse_kmh_s": float(np.sqrt(np.mean(r ** 2))), "n": int(tramo.sum())}
    return planta


class Perfil:
    def __init__(self, corrida):
        p = np.genfromtxt(CORRIDAS / corrida / "perfil.csv", delimiter=",", names=True)
        self.v = p["v_perfil_kmh"]
        self.ds = p["s_m"][-1] / (len(p) - 1)
        self.largo = p["s_m"][-1] + self.ds

    def indice(self, s):
        return int(round((s % self.largo) / self.ds)) % len(self.v)

    def referencia(self, s, v_kmh):
        pasos = max(1, int(round((PREVIEW_M + PREVIEW_GANANCIA * max(0.0, v_kmh / 3.6)) / self.ds)))
        return float(self.v[(self.indice(s) + np.arange(pasos + 1)) % len(self.v)].min())


def aceleracion(planta, u, v):
    p = planta["gas"] if u >= 0 else planta["freno"]
    return p["a"] * u - p["c"] * v + p["d"]


def simular(planta, perfil, lon, s0, v0, t_max):
    pid = PIDLongitudinalController(kp=lon["kp"], ki=lon["ki"], kd=lon["kd"], dt=TS,
                                    limite_integral=lon["limite_integral"])
    ped = lon["pedal"]
    conf = PedalShaper(TS, ped["throttle_rise_s"], ped["throttle_fall_s"], ped["brake_rise_s"], ped["brake_fall_s"])
    cola = [0.0] * RETARDO_CICLOS
    s, v, filas = s0, v0, []
    for n in range(int(t_max / TS)):
        vr = perfil.referencia(s, v)
        u = conf.shape(pid.compute_control(v, vr))
        cola.append(u)
        filas.append((n * TS, v, vr, perfil.v[perfil.indice(s)], u, s % perfil.largo))
        v = max(0.0, v + aceleracion(planta, cola.pop(0), v) * TS)
        s += v / 3.6 * TS
    return {c: np.array(x) for c, x in zip(COLUMNAS, zip(*filas))}


def metricas(T, largo):
    s = T["s_cg_m"]
    cruces = np.where((s[1:] < 500) & (s[:-1] > largo - 500))[0] + 1
    if len(cruces) < 2:
        return None
    a, b = cruces[0], cruces[1]
    e = T["v_kmh"][a:b] - T["v_ref_kmh"][a:b]
    sobre = T["v_kmh"][a:b] - T["v_perfil_kmh"][a:b]
    u = T["u_pedal"][a:b]
    signo = np.sign(u[np.abs(u) > 1e-6])
    return {
        "tiempo_vuelta_s": float(T["t_s"][b] - T["t_s"][a]),
        "rmse_v_menos_vref_kmh": float(np.sqrt(np.mean(e ** 2))),
        "error_medio_gas_kmh": float(e[u > 1e-6].mean()),
        "error_medio_freno_kmh": float(e[u < -1e-6].mean()),
        "ciclos_sobre_perfil_pct": float(np.mean(sobre > 0) * 100),
        "sobrevelocidad_p95_kmh": float(np.percentile(sobre[sobre > 0], 95)),
        "sobrevelocidad_max_kmh": float(sobre.max()),
        "ciclos_mas_5_kmh": int(np.sum(sobre > 5)),
        "cambios_gas_freno": int(np.sum(np.diff(signo) != 0)),
    }


def main():
    SALIDA.mkdir(exist_ok=True)
    planta = ajustar(ENTRENAMIENTO)
    perfil = Perfil(ENTRENAMIENTO)
    validacion = {}
    for corrida, config in VALIDACION.items():
        man = json.load(open(CORRIDAS / corrida / "manifiesto.json", encoding="utf-8"))
        lon = man["config"]["longitudinal"]
        T = cargar(corrida)
        sim = simular(planta, perfil, lon, T["s_cg_m"][0], T["v_kmh"][0], T["t_s"][-1] + 5)
        validacion[corrida] = {"config": config, "ganancias": {k: lon[k] for k in ("kp", "ki", "kd", "limite_integral")},
                               "real": metricas(T, perfil.largo), "simulada": metricas(sim, perfil.largo)}
        print(f"\n{corrida} ({config})")
        for k in validacion[corrida]["real"]:
            print(f"  {k:28s} real {validacion[corrida]['real'][k]:9.2f}  simulada {validacion[corrida]['simulada'][k]:9.2f}")

    T0 = cargar(ENTRENAMIENTO)
    info = {
        "descripcion": __doc__.strip().splitlines()[0],
        "corrida_entrenamiento": ENTRENAMIENTO,
        "ts_s": TS, "retardo_ciclos": RETARDO_CICLOS,
        "planta": planta, "validacion_lazo_cerrado": validacion,
        "perfil": {"largo_m": perfil.largo, "ds_m": perfil.ds, "preview_m": PREVIEW_M, "preview_ganancia_s": PREVIEW_GANANCIA},
        "condicion_inicial": {"s0_m": float(T0["s_cg_m"][0]), "v0_kmh": float(T0["v_kmh"][0])},
    }
    json.dump(info, open(SALIDA / "planta_longitudinal.json", "w", encoding="utf-8"), indent=2, ensure_ascii=False)
    g, f = planta["gas"], planta["freno"]
    savemat(SALIDA / "datos_planta_pid.mat", {
        "Ts": TS, "v_perfil_kmh": perfil.v.reshape(-1, 1), "ds_m": perfil.ds, "largo_m": perfil.largo,
        "preview_m": PREVIEW_M, "preview_ganancia": PREVIEW_GANANCIA,
        "a_g": g["a"], "c_g": g["c"], "d_g": g["d"], "a_f": f["a"], "c_f": f["c"], "d_f": f["d"],
        "s0_m": info["condicion_inicial"]["s0_m"], "v0_kmh": info["condicion_inicial"]["v0_kmh"],
    })
    print("\nplanta", json.dumps(planta, indent=1))


if __name__ == "__main__":
    main()
