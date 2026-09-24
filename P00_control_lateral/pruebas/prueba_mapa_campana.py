"""
Pruebas del análisis de la campaña. Datos sintéticos, no tocan corridas reales.

    python P00_control_lateral/pruebas/prueba_mapa_campana.py
"""
import sys
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from analisis import mapa_campana as mc  # noqa: E402

RESULTADOS = []


def verificar(nombre, ok, detalle=""):
    RESULTADOS.append((nombre, bool(ok)))
    print(f"[{'OK' if ok else 'FALLA'}] {nombre}" + (f" {detalle}" if detalle else ""))


def fila(controlador, sesion, rmse, tasa=0.02, tc=0.0, completada=True):
    return {"id_corrida": f"{controlador}_{sesion}", "carpeta": "x", "controlador": controlador,
            "perfil": "nominal", "sesion": sesion, "id_plan": "", "inicio": "",
            "vuelta_completada": completada,
            "grupos": {"media": {"rmse_e_y_cg_m": rmse, "rms_tasa_delta_rad_s": tasa,
                                 "pct_tc_mayor_50ms": tc}}}


def main():
    # Holm sobre un caso calculable a mano.
    p = {"a": 0.01, "b": 0.02, "c": 0.03}
    aj = mc.holm(p)
    # Con m igual a 3, los p ajustados son 3p, 2p y 1p, y después se fuerza
    # que la secuencia no decrezca, de modo que el tercero hereda el segundo.
    verificar("Holm multiplica por el número de pruebas restantes",
              abs(aj["a"] - 0.03) < 1e-12 and abs(aj["b"] - 0.04) < 1e-12,
              str({k: round(v, 4) for k, v in aj.items()}))
    verificar("Holm fuerza que los ajustados no decrezcan",
              abs(aj["c"] - 0.04) < 1e-12 and aj["a"] <= aj["b"] <= aj["c"],
              str(round(aj["c"], 4)))
    verificar("Holm acota en uno", max(mc.holm({"a": 0.5, "b": 0.6}).values()) <= 1.0)
    verificar("Holm con familia vacía devuelve vacío", mc.holm({}) == {})

    # Bootstrap.
    bajo, alto = mc.bootstrap_diferencia([0.10] * 8)
    verificar("bootstrap de datos idénticos da intervalo nulo",
              abs(bajo - 0.10) < 1e-9 and abs(alto - 0.10) < 1e-9, f"[{bajo}, {alto}]")
    dif = list(np.linspace(0.05, 0.15, 10))
    bajo, alto = mc.bootstrap_diferencia(dif)
    verificar("el intervalo contiene la media", bajo < np.mean(dif) < alto, f"[{bajo:.4f}, {alto:.4f}]")
    verificar("el intervalo es reproducible con la misma semilla",
              mc.bootstrap_diferencia(dif) == mc.bootstrap_diferencia(dif))
    verificar("con una sola sesión no hay intervalo", mc.bootstrap_diferencia([0.1]) == (None, None))

    cond = {"max_pct_tc_sobre_periodo": 1.0, "max_vueltas_fallidas": 2,
            "banda_bajo_costo": 1.5, "banda_actividad_superior": 3.0}
    umbral = {"pure_pursuit": 0.02, "stanley": 0.015}

    # Celda donde el MPC mejora de forma clara y estable.
    filas = []
    for s in range(1, 11):
        filas.append(fila("mpc_cinematico", s, 0.08 + 0.001 * s, tasa=0.020))
        filas.append(fila("stanley", s, 0.14 + 0.001 * s, tasa=0.018))
        filas.append(fila("pure_pursuit", s, 0.22 + 0.001 * s, tasa=0.017))
    celda = mc.analizar_celda(filas, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)
    verificar("cuenta las sesiones completas", celda["sesiones_completas"] == 10)
    verificar("resume los tres controladores", len(celda["por_controlador"]) == 3)
    verificar("corre Friedman", "p" in celda.get("friedman", {}))
    comp = celda["comparaciones"]["stanley"]
    verificar("mide la reducción media", abs(comp["reduccion_media_m"] - 0.06) < 1e-9,
              str(round(comp["reduccion_media_m"], 4)))
    verificar("corre Wilcoxon con 10 sesiones", "wilcoxon_p" in comp)
    verificar("declara que supera el umbral", comp["supera_umbral"] is True)
    verificar("mide la razón de esfuerzo",
              abs(comp["razon_esfuerzo_mpc_sobre_geometrico"] - 0.020 / 0.018) < 1e-9,
              f"{comp['razon_esfuerzo_mpc_sobre_geometrico']:.3f}")
    verificar("etiqueta la mejora como de bajo costo si la razón no pasa de 1.5",
              comp["etiqueta"] == "mejora practica de bajo costo", comp["etiqueta"])

    # Celda donde la reducción no alcanza el umbral.
    filas2 = []
    for s in range(1, 11):
        filas2.append(fila("mpc_cinematico", s, 0.130 + 0.001 * s))
        filas2.append(fila("stanley", s, 0.140 + 0.001 * s))
        filas2.append(fila("pure_pursuit", s, 0.145 + 0.001 * s))
    celda2 = mc.analizar_celda(filas2, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)
    verificar("no declara mejora si la reducción es menor al umbral",
              celda2["comparaciones"]["stanley"]["supera_umbral"] is False,
              str(round(celda2["comparaciones"]["stanley"]["reduccion_media_m"], 4)))
    verificar("sin mejora la etiqueta lo dice",
              celda2["comparaciones"]["stanley"]["etiqueta"] == "sin mejora practica")

    # Condición de tiempo real y de suavidad.
    # Copia propia, si no la mutación contaminaría las comprobaciones siguientes.
    filas3 = [dict(f, grupos={k: dict(v) for k, v in f["grupos"].items()}) for f in filas]
    for f in filas3:
        if f["controlador"] == "mpc_cinematico":
            f["grupos"]["media"]["pct_tc_mayor_50ms"] = 5.0
            f["grupos"]["media"]["rms_tasa_delta_rad_s"] = 0.050
    celda3 = mc.analizar_celda(filas3, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)
    c3 = celda3["comparaciones"]["stanley"]
    verificar("detecta el incumplimiento de tiempo real", c3["cumple_tiempo_real"] is False)
    verificar("una celda sin tiempo real queda no evaluable",
              c3["etiqueta"] == "no evaluable", c3["etiqueta"])
    verificar("la reducción se mide igual aunque la celda no sea evaluable",
              c3["supera_umbral"] is True)

    # El esfuerzo no bloquea, solo cambia la etiqueta.
    filas5 = [dict(f, grupos={k: dict(v) for k, v in f["grupos"].items()}) for f in filas]
    for f in filas5:
        if f["controlador"] == "mpc_cinematico":
            f["grupos"]["media"]["rms_tasa_delta_rad_s"] = 0.036   # razón 2.0 contra Stanley
    c5 = mc.analizar_celda(filas5, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)["comparaciones"]["stanley"]
    verificar("con razón entre las bandas etiqueta mayor actividad",
              c5["etiqueta"] == "mejora practica con mayor actividad de direccion", c5["etiqueta"])
    for f in filas5:
        if f["controlador"] == "mpc_cinematico":
            f["grupos"]["media"]["rms_tasa_delta_rad_s"] = 0.080   # razón 4.4
    c6 = mc.analizar_celda(filas5, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)["comparaciones"]["stanley"]
    verificar("con razón sobre la banda alta etiqueta actividad muy superior",
              c6["etiqueta"] == "mejora practica con actividad muy superior", c6["etiqueta"])
    verificar("el esfuerzo no anula la mejora", c6["supera_umbral"] is True)

    # Sesiones incompletas.
    filas4 = [f for f in filas if not (f["sesion"] == 3 and f["controlador"] == "stanley")]
    celda4 = mc.analizar_celda(filas4, "media", "nominal", "rmse_e_y_cg_m", umbral, cond)
    verificar("descarta la sesión sin los tres controladores",
              celda4["sesiones_completas"] == 9, str(celda4["sesiones_completas"]))

    # Celda sin datos.
    vacia = mc.analizar_celda(filas, "alta", "rapido", "rmse_e_y_cg_m", umbral, cond)
    verificar("una celda sin datos no rompe", vacia["por_controlador"] == {})

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
