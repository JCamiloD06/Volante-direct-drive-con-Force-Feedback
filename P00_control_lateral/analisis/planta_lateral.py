"""
Planta lateral identificada con telemetría y validada en lazo cerrado.

Sirve para acotar los rangos de búsqueda de la fase 4 sin gastar vueltas en
pista. No reemplaza ninguna corrida, solo descarta de antemano las zonas del
espacio de parámetros que oscilan o saturan la dirección.

Modelo, bicicleta cinemático sobre el eje trasero con ganancia y retardo medidos.
    δ_ef'   = (δ - δ_ef) / tau_dir          retardo de primer orden de la dirección
    x_tras' = v cos ψ
    z_tras' = v sin ψ
    ψ'      = g_psi v tan(δ_ef) / L
El eje delantero es el trasero más L en la dirección de ψ. g_psi recoge todo
lo que el modelo cinemático no explica, deriva de las llantas, transferencia
de carga y la conversión del eje de mando a ángulo de rueda. La velocidad no
se simula, se toma medida de la vuelta real en función de la distancia
recorrida, para aislar la dinámica lateral de la longitudinal.

El lazo simulado usa las mismas piezas que la corrida real, Proyector,
InterfazDireccion, entrada_progresiva y el controlador tal cual, con un ciclo
de retardo entre el comando y su efecto.

Uso
    python analisis/planta_lateral.py
Escribe sintonia_fase4/planta_lateral.json con la identificación y la
validación de las tres vueltas del 2026-09-16.
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))
from controladores.mpc_cinematico import MPCCinematico  # noqa: E402
from controladores.base import EntradaLateral  # noqa: E402
from controladores.pure_pursuit import PurePursuit  # noqa: E402
from controladores.stanley import Stanley  # noqa: E402
from plataforma import procedencia  # noqa: E402
from plataforma.actuador import InterfazDireccion, entrada_progresiva  # noqa: E402
from plataforma.angulos import envolver  # noqa: E402
from plataforma.trazada import Proyector, Trazada  # noqa: E402

CORRIDAS = RAIZ.parent / "data" / "raw" / "p00" / "corridas"
SALIDA = RAIZ / "sintonia_fase4"
TS = 0.05
RETARDO_CICLOS = 1

VUELTAS = {
    "stanley": "verificacion_s00_stanley_conservador_20260916_211412",
    "pure_pursuit": "verificacion_s00_pure_pursuit_conservador_20260916_212145",
    "mpc_cinematico": "verificacion_s00_mpc_cinematico_conservador_20260916_212712",
}
ENTRENAMIENTO = "stanley"

COLUMNAS = ["ciclo", "t_s", "v_kmh", "psi_rad", "delta_aplicado_rad", "tasa_guinada_local",
            "x_tras", "z_tras", "s_tras_m", "e_y_tras_m", "e_psi_tras_rad", "e_y_cg_m",
            "L_usada_m", "s_cg_m", "ruedas_fuera"]


def filtrar(x, tau):
    """Retardo de primer orden discreto con periodo TS."""
    if tau <= 0.0:
        return np.asarray(x, dtype=float).copy()
    a = TS / (tau + TS)
    y = np.zeros_like(x, dtype=float)
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i - 1] - y[i - 1])
    return y


def cargar(corrida):
    with open(CORRIDAS / corrida / "telemetria.csv", encoding="utf-8") as f:
        a = np.array([[r[c] for c in COLUMNAS] for r in csv.DictReader(f)], dtype=float)
    T = {c: a[:, i] for i, c in enumerate(COLUMNAS)}
    T["manifiesto"] = json.load(open(CORRIDAS / corrida / "manifiesto.json", encoding="utf-8"))
    return T


def ventana_vuelta(T):
    r = T["manifiesto"]["resumen"]
    k = (T["ciclo"] >= r["cruce_inicio"]["ciclo"]) & (T["ciclo"] <= r["cruce_fin"]["ciclo"])
    return k


def identificar(T):
    """g_psi por mínimos cuadrados sin término independiente, con el retardo aplicado."""
    v = T["v_kmh"] / 3.6
    n = len(v) - RETARDO_CICLOS
    # La guiñada se deriva de psi_rad. La columna tasa_guinada_local viene del
    # marco local de Assetto Corsa y tiene el signo contrario, correlación de
    # menos 0.9998 con la derivada de psi, verificado el 2026-09-18.
    medida = np.gradient(np.unwrap(T["psi_rad"]), T["t_s"])[RETARDO_CICLOS:]
    # El ángulo de mando no se traduce en giro instantáneo. Se barre la
    # constante de un retardo de primer orden y se toma la de mejor ajuste.
    mejor = None
    for tau in np.arange(0.0, 0.31, 0.01):
        d_ef = filtrar(T["delta_aplicado_rad"], tau)
        modelo = v[RETARDO_CICLOS:] * np.tan(d_ef[:n]) / T["L_usada_m"][RETARDO_CICLOS:]
        valido = (v[RETARDO_CICLOS:] > 5.0) & np.isfinite(medida) & np.isfinite(modelo)
        x, y = modelo[valido], medida[valido]
        g = float(x @ y / (x @ x))
        r = y - g * x
        r2 = 1.0 - float(r @ r) / float(((y - y.mean()) ** 2).sum())
        if mejor is None or r2 > mejor[2]:
            mejor = (float(tau), g, r2, float(np.sqrt((r ** 2).mean())), int(valido.sum()))
    tau_dir, g, r2, rmse, n_psi = mejor
    # Chequeo de la otra relación cinemática, e_y' contra v sen(e_psi).
    ey = T["e_y_tras_m"]
    dey = np.gradient(ey, T["t_s"])
    modelo_ey = v * np.sin(T["e_psi_tras_rad"])
    m2 = (v > 5.0) & np.isfinite(dey)
    g_ey = float(modelo_ey[m2] @ dey[m2] / (modelo_ey[m2] @ modelo_ey[m2]))
    r_ey = dey[m2] - g_ey * modelo_ey[m2]
    r2_ey = 1.0 - float(r_ey @ r_ey) / float(((dey[m2] - dey[m2].mean()) ** 2).sum())
    return {"g_psi": g, "tau_dir_s": tau_dir, "r2_psi": r2, "n_psi": n_psi,
            "rmse_psi_rad_s": rmse,
            "g_ey": g_ey, "r2_ey": r2_ey, "n_ey": int(m2.sum()),
            "_nota_ey": ("Regresión atenuada por el ruido de e_psi, que es la variable "
                         "explicativa. La relación cinemática no queda verificada con este ajuste "
                         "y no se usa en la simulación, que integra posición y orientación.")}


class VelocidadMedida:
    """Velocidad de la vuelta real en función de la distancia recorrida."""

    def __init__(self, T, largo):
        k = ventana_vuelta(T)
        s = np.unwrap(T["s_tras_m"][k], period=largo)
        v = T["v_kmh"][k]
        orden = np.argsort(s)
        self.s, self.v = s[orden], v[orden]

    def __call__(self, s):
        return float(np.interp(s, self.s, self.v, left=self.v[0], right=self.v[-1]))


def construir_controlador(nombre, cfg):
    p = cfg["controladores"][nombre]
    d = cfg["direccion"]
    if nombre == "pure_pursuit":
        return PurePursuit(p)
    if nombre == "stanley":
        return Stanley(p)
    return MPCCinematico(p, TS, d["delta_max_rad"], d["tasa_max_rad_s"])


def simular(controlador, cfg, trazada, velocidad, estado0, pasos):
    d = cfg["direccion"]
    interfaz = InterfazDireccion(d["delta_max_rad"], d["tasa_max_rad_s"], TS)
    proy_tras, proy_del = Proyector(trazada), Proyector(trazada)
    g, tau_dir = cfg["_g_psi"], cfg["_tau_dir_s"]
    alfa = TS / (tau_dir + TS) if tau_dir > 0.0 else 1.0
    x, z, psi, L = estado0["x"], estado0["z"], estado0["psi"], estado0["L"]
    delta_ef = 0.0
    cola = [0.0] * RETARDO_CICLOS
    s_rec, filas = 0.0, []
    for k in range(pasos):
        v_kmh = velocidad(estado0["s0"] + s_rec)
        v = v_kmh / 3.6
        xd, zd = x + L * math.cos(psi), z + L * math.sin(psi)
        p_tr, p_de = proy_tras.proyectar(x, z), proy_del.proyectar(xd, zd)
        entrada = EntradaLateral(
            trazada=trazada, v_ms=v, L=L, psi=psi, x_tras=x, z_tras=z, x_del=xd, z_del=zd,
            idx_tras=p_tr.idx, e_y_tras=p_tr.e_y, e_psi_tras=envolver(psi - p_tr.phi),
            idx_del=p_de.idx, e_y_del=p_de.e_y, e_psi_del=envolver(psi - p_de.phi),
            delta_prev=interfaz.delta_prev)
        sal = controlador.calcular(entrada)
        escalado, _ = entrada_progresiva(sal.delta, v_kmh,
                                         d["velocidad_minima_direccion_kmh"],
                                         d["velocidad_plena_direccion_kmh"])
        aplicado, sat_mag, sat_tasa = interfaz.aplicar(escalado, k * TS)
        cola.append(aplicado)
        efectivo = cola.pop(0)
        filas.append((k * TS, p_tr.e_y, envolver(psi - p_tr.phi), aplicado, v_kmh, s_rec,
                      float(sat_mag), float(sat_tasa)))
        delta_ef += alfa * (efectivo - delta_ef)
        psi = envolver(psi + g * v * math.tan(delta_ef) / L * TS)
        x += v * math.cos(psi) * TS
        z += v * math.sin(psi) * TS
        s_rec += v * TS
    nombres = ["t_s", "e_y", "e_psi", "delta", "v_kmh", "s_rec", "sat_mag", "sat_tasa"]
    return {c: np.array(v) for c, v in zip(nombres, zip(*filas))}


def lazo_abierto(T, g, tau_dir, ventana_ciclos=40, paso=40):
    """
    Repite tramos de 2 s con el ángulo medido y compara con la trayectoria real.
    Mide el error del modelo sin realimentación, que es el techo de lo que la
    simulación en lazo cerrado puede predecir.
    """
    x, z = T["x_tras"], T["z_tras"]
    psi = np.unwrap(T["psi_rad"])
    v, d, L = T["v_kmh"] / 3.6, T["delta_aplicado_rad"], T["L_usada_m"]
    a = TS / (tau_dir + TS) if tau_dir > 0 else 1.0
    errores, err_psi = [], []
    for i0 in range(200, len(x) - ventana_ciclos - 1, paso):
        if v[i0] < 10.0:
            continue
        xs, zs, ps, de = x[i0], z[i0], psi[i0], d[i0]
        for j in range(i0, i0 + ventana_ciclos):
            de += a * (d[j - 1] - de)
            ps += g * v[j] * math.tan(de) / L[j] * TS
            xs += v[j] * math.cos(ps) * TS
            zs += v[j] * math.sin(ps) * TS
        j = i0 + ventana_ciclos
        dx, dz = x[j] - x[i0], z[j] - z[i0]
        n = math.hypot(dx, dz) or 1.0
        errores.append(((xs - x[j]) * (-dz) + (zs - z[j]) * dx) / n)
        err_psi.append(ps - psi[j])
    return {"ventana_s": ventana_ciclos * TS, "n": len(errores),
            "error_lateral_rms_m": float(np.sqrt(np.mean(np.square(errores)))),
            "error_lateral_max_m": float(np.max(np.abs(errores))),
            "error_psi_rms_deg": float(math.degrees(np.sqrt(np.mean(np.square(err_psi)))))}


def metricas(R, largo_vuelta):
    k = R["s_rec"] <= largo_vuelta
    e, dl = R["e_y"][k], R["delta"][k]
    return {"rmse_e_y_m": float(np.sqrt((e ** 2).mean())),
            "max_abs_e_y_m": float(np.abs(e).max()),
            "rms_tasa_rad_s": float(np.sqrt(((np.diff(dl) / TS) ** 2).mean())),
            "ciclos_sat_magnitud": int(R["sat_mag"][k].sum()),
            "ciclos_sat_tasa": int(R["sat_tasa"][k].sum()),
            "ciclos": int(k.sum())}


def main():
    SALIDA.mkdir(exist_ok=True)
    T_ent = cargar(VUELTAS[ENTRENAMIENTO])
    ident = identificar(T_ent)
    print(f"identificación con {VUELTAS[ENTRENAMIENTO]}")
    print(f"  τ dirección = {ident['tau_dir_s']:.2f} s")
    print(f"  ψ' = g v tan(δ_ef)/L   g = {ident['g_psi']:.4f}  R2 = {ident['r2_psi']:.3f}  "
          f"RMSE {ident['rmse_psi_rad_s']:.4f} rad/s  n = {ident['n_psi']}")
    print(f"  e_y' = g v sen(e_ψ)  g = {ident['g_ey']:.4f}  R2 = {ident['r2_ey']:.3f}  n = {ident['n_ey']}")

    cfg = json.load(open(RAIZ / "configs" / "base.json", encoding="utf-8"))
    cfg["_g_psi"] = ident["g_psi"]
    cfg["_tau_dir_s"] = ident["tau_dir_s"]
    trazada = Trazada(procedencia.RAIZ_REPO / cfg["trazada"]["csv"],
                      cfg["trazada"]["suavizado_curvatura_m"])
    largo = trazada.total_length

    validacion = {}
    for nombre, corrida in VUELTAS.items():
        T = cargar(corrida)
        k = ventana_vuelta(T)
        i0 = int(np.argmax(k))
        velocidad = VelocidadMedida(T, largo)
        estado0 = {"x": T["x_tras"][i0], "z": T["z_tras"][i0], "psi": T["psi_rad"][i0],
                   "L": float(np.median(T["L_usada_m"][k])), "s0": T["s_tras_m"][i0]}
        R = simular(construir_controlador(nombre, cfg), cfg, trazada, velocidad, estado0, int(k.sum()))
        real = {"rmse_e_y_m": float(np.sqrt((T["e_y_tras_m"][k] ** 2).mean())),
                "max_abs_e_y_m": float(np.abs(T["e_y_tras_m"][k]).max()),
                "rms_tasa_rad_s": float(np.sqrt(((np.diff(T["delta_aplicado_rad"][k]) / TS) ** 2).mean())),
                "ciclos": int(k.sum())}
        sim = metricas(R, largo)
        abierto = lazo_abierto(T, ident["g_psi"], ident["tau_dir_s"])
        validacion[nombre] = {"corrida": corrida, "real": real, "simulada": sim,
                              "lazo_abierto": abierto}
        print(f"\n{nombre}")
        for campo in ("rmse_e_y_m", "max_abs_e_y_m", "rms_tasa_rad_s"):
            print(f"  {campo:18s} real {real[campo]:8.4f}  simulada {sim[campo]:8.4f}")
        print(f"  lazo abierto 2 s   error lateral RMS {abierto['error_lateral_rms_m']:.3f} m  "
              f"máx {abierto['error_lateral_max_m']:.2f} m  error ψ RMS {abierto['error_psi_rms_deg']:.2f}°")

    json.dump({"descripcion": __doc__.strip().splitlines()[0],
               "conclusion": ("La identificación de la guiñada es sólida, pero la simulación en lazo "
                              "cerrado no reproduce el error lateral. Subestima el RMSE de Pure Pursuit y "
                              "Stanley en cerca de la mitad y hace inestable al MPC, que en pista no "
                              "satura la tasa en ningún ciclo. En lazo abierto el modelo se desvía más de "
                              "un metro en dos segundos, muy por encima de las diferencias que se quieren "
                              "medir. NO SIRVE para filtrar rangos de sintonía por error lateral."),
               "corrida_entrenamiento": VUELTAS[ENTRENAMIENTO],
               "ts_s": TS, "retardo_ciclos": RETARDO_CICLOS,
               "identificacion": ident, "validacion": validacion},
              open(SALIDA / "planta_lateral.json", "w", encoding="utf-8"), indent=2, ensure_ascii=False)


if __name__ == "__main__":
    main()
