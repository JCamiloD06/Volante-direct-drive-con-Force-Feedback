"""
Mapa de dominio de operación y contraste estadístico de la campaña.

Aplica las decisiones 1, 2, 4 y 5 sin decidir nada aquí. Toda regla y todo
umbral se leen de la configuración y de la salida del piloto.

Unidad de análisis. La vuelta dentro de una celda, decisión 4.3, nunca el ciclo
de 50 ms, porque las muestras consecutivas no son independientes. Cada celda es
un perfil de velocidad por una región de curvatura, 3 por 3, decisión 3.3.

Contraste. Friedman sobre las 3 estrategias con la sesión como bloque, seguido
de Wilcoxon pareado del MPC contra cada geométrico. Corrección de Holm en dos
familias de 9 celdas, una por geométrico, decisión 4.4. Intervalo bootstrap del
95 por ciento de la diferencia, remuestreando sesiones completas.

Criterio en tres capas, fijado el 2026-09-20 en la sección criterio_mapa de la
configuración. Primero factibilidad, tiempo real y vueltas fallidas, que decide
si la celda es evaluable. Después la decisión de mejora, que es la única que
produce veredicto y usa solo el RMSE de e_y con su umbral, su intervalo
bootstrap y Holm. Por último el esfuerzo de dirección, que pasó de condición
que bloquea a métrica reportada, y etiqueta cada celda según la razón entre el
RMS de tasa del MPC y el del geométrico.

Uso desde la raíz del repositorio
    python P00_control_lateral/analisis/mapa_campana.py
    python P00_control_lateral/analisis/mapa_campana.py --fase piloto --perfiles nominal
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import stats

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from analisis import metricas_vuelta  # noqa: E402
from plataforma import procedencia  # noqa: E402

GEOMETRICOS = ("pure_pursuit", "stanley")
MPC = "mpc_cinematico"
REGIONES = ("baja", "media", "alta")
PERFILES = ("conservador", "nominal", "rapido")
SALIDA = RAIZ_P00 / "resultados_campana"


def cargar_umbrales(ruta):
    """Umbrales de mejora práctica medidos en el piloto, decisión 1.2."""
    if not Path(ruta).exists():
        return None
    d = json.load(open(ruta, encoding="utf-8"))
    return {g: v["umbral_m"] for g, v in d.get("umbrales", {}).items()}


def recolectar(cfg, fase, perfiles, recalcular=False):
    """Métricas por vuelta de todas las corridas de la fase pedida."""
    directorio = procedencia.RAIZ_REPO / cfg["salida"]["directorio_corridas"]
    filas = []
    for carpeta in sorted(directorio.glob("*/")):
        manifiesto = carpeta / "manifiesto.json"
        if not manifiesto.exists():
            continue
        man = json.load(open(manifiesto, encoding="utf-8"))
        meta = man.get("meta", {})
        if meta.get("fase") != fase or meta.get("perfil") not in perfiles:
            continue
        ruta_metricas = carpeta / "metricas.json"
        if recalcular or not ruta_metricas.exists():
            m = metricas_vuelta.procesar(carpeta, cfg)
        else:
            m = json.load(open(ruta_metricas, encoding="utf-8"))
        filas.append({
            "id_corrida": m.get("id_corrida"), "carpeta": carpeta.name,
            "controlador": meta.get("controlador"), "perfil": meta.get("perfil"),
            "sesion": meta.get("sesion"), "id_plan": meta.get("id_plan") or "",
            "inicio": man.get("inicio", ""),
            "vuelta_completada": bool((man.get("resumen") or {}).get("vuelta_completada")),
            "grupos": m.get("grupos", {}),
        })
    # Un identificador repetido significa reintento. Se usa el último.
    ultimos = {}
    for f in sorted(filas, key=lambda x: x["inicio"]):
        clave = f["id_plan"] or f["id_corrida"]
        ultimos[clave] = f
    return list(ultimos.values())


def valor(fila, region, campo):
    g = fila["grupos"].get(region) or {}
    v = g.get(campo)
    return float(v) if v is not None else None


def bootstrap_diferencia(pares, n=10000, semilla=0):
    """
    Intervalo del 95 por ciento de la diferencia media, remuestreando sesiones
    completas, decisión 4.3. pares es una lista de diferencias por sesión.
    """
    if len(pares) < 2:
        return None, None
    rng = np.random.default_rng(semilla)
    a = np.asarray(pares, dtype=float)
    muestras = rng.choice(a, size=(n, a.size), replace=True).mean(axis=1)
    return float(np.percentile(muestras, 2.5)), float(np.percentile(muestras, 97.5))


def holm(pvalores):
    """Corrección de Holm dentro de una familia. Devuelve los p ajustados."""
    if not pvalores:
        return {}
    items = sorted(pvalores.items(), key=lambda kv: kv[1])
    m = len(items)
    ajustados = {}
    previo = 0.0
    for i, (clave, p) in enumerate(items):
        valor_ajustado = min(1.0, max(previo, (m - i) * p))
        ajustados[clave] = valor_ajustado
        previo = valor_ajustado
    return ajustados


def etiquetar(comp, razon, evaluable, cfg_cond):
    """
    Etiqueta de la celda. La mejora la decide solo el error lateral, y el
    esfuerzo de dirección califica esa mejora con las bandas declaradas.
    """
    if not evaluable:
        return "no evaluable"
    if comp.get("supera_umbral") is not True:
        return "sin mejora practica"
    if razon <= cfg_cond["banda_bajo_costo"]:
        return "mejora practica de bajo costo"
    if razon <= cfg_cond["banda_actividad_superior"]:
        return "mejora practica con mayor actividad de direccion"
    return "mejora practica con actividad muy superior"


def analizar_celda(filas, region, perfil, campo_rmse, umbral, cfg_cond):
    """
    Una celda del mapa. Devuelve el resumen por controlador, las pruebas y el
    veredicto de mejora práctica contra cada geométrico.
    """
    por_sesion = {}
    for f in filas:
        if f["perfil"] != perfil:
            continue
        v = valor(f, region, campo_rmse)
        if v is None:
            continue
        por_sesion.setdefault(f["sesion"], {})[f["controlador"]] = f

    completas = {s: d for s, d in por_sesion.items()
                 if all(c in d for c in (MPC,) + GEOMETRICOS)}
    celda = {"perfil": perfil, "region": region, "sesiones": len(por_sesion),
             "sesiones_completas": len(completas), "por_controlador": {}, "comparaciones": {}}

    for c in (MPC,) + GEOMETRICOS:
        vals = [valor(d[c], region, campo_rmse) for d in completas.values() if c in d]
        fallidas = sum(1 for d in completas.values() if c in d and not d[c]["vuelta_completada"])
        if not vals:
            continue
        celda["por_controlador"][c] = {
            "n": len(vals), "rmse_medio_m": float(np.mean(vals)),
            "rmse_mediana_m": float(np.median(vals)),
            "rms_tasa_media_rad_s": float(np.mean([valor(d[c], region, "rms_tasa_delta_rad_s") or np.nan
                                                   for d in completas.values() if c in d])),
            "pct_tc_mayor_50ms_max": float(np.nanmax([valor(d[c], region, "pct_tc_mayor_50ms") or 0.0
                                                      for d in completas.values() if c in d])),
            "vueltas_fallidas": fallidas,
        }

    if len(completas) >= 2 and len(celda["por_controlador"]) == 3:
        matriz = [[valor(d[c], region, campo_rmse) for c in (MPC,) + GEOMETRICOS]
                  for d in completas.values()]
        try:
            fr = stats.friedmanchisquare(*np.array(matriz).T)
            celda["friedman"] = {"estadistico": float(fr.statistic), "p": float(fr.pvalue)}
        except ValueError as e:
            celda["friedman"] = {"error": str(e)}

    mpc = celda["por_controlador"].get(MPC)
    for g in GEOMETRICOS:
        geo = celda["por_controlador"].get(g)
        if not (mpc and geo) or not completas:
            continue
        dif = [valor(d[g], region, campo_rmse) - valor(d[MPC], region, campo_rmse)
               for d in completas.values()]
        bajo, alto = bootstrap_diferencia(dif)
        comp = {"reduccion_media_m": float(np.mean(dif)),
                "ic95_bootstrap_m": [bajo, alto],
                "umbral_m": umbral.get(g) if umbral else None,
                "n_sesiones": len(dif)}
        if len(dif) >= 6:
            w = stats.wilcoxon([valor(d[MPC], region, campo_rmse) for d in completas.values()],
                               [valor(d[g], region, campo_rmse) for d in completas.values()])
            comp["wilcoxon_p"] = float(w.pvalue)
        # Capa 1, factibilidad. Decide si la celda es evaluable.
        evaluable = (mpc["pct_tc_mayor_50ms_max"] <= cfg_cond["max_pct_tc_sobre_periodo"]
                     and mpc["vueltas_fallidas"] <= cfg_cond["max_vueltas_fallidas"]
                     and geo["vueltas_fallidas"] <= cfg_cond["max_vueltas_fallidas"])
        comp["evaluable"] = bool(evaluable)
        comp["cumple_tiempo_real"] = bool(mpc["pct_tc_mayor_50ms_max"]
                                          <= cfg_cond["max_pct_tc_sobre_periodo"])
        # Capa 3, esfuerzo reportado. No bloquea, etiqueta.
        razon = mpc["rms_tasa_media_rad_s"] / geo["rms_tasa_media_rad_s"]
        comp["razon_esfuerzo_mpc_sobre_geometrico"] = float(razon)
        # Capa 2, la única que produce veredicto.
        if comp["umbral_m"] is not None and bajo is not None:
            comp["supera_umbral"] = bool(comp["reduccion_media_m"] > comp["umbral_m"]
                                         and bajo > comp["umbral_m"])
        comp["etiqueta"] = etiquetar(comp, razon, evaluable, cfg_cond)
        celda["comparaciones"][g] = comp
    return celda


def main():
    ap = argparse.ArgumentParser(description="Mapa de dominio de operación de P00")
    ap.add_argument("--config", default=str(RAIZ_P00 / "configs" / "base.json"))
    ap.add_argument("--fase", default="campana")
    ap.add_argument("--perfiles", nargs="*", default=list(PERFILES))
    ap.add_argument("--umbrales", default=str(RAIZ_P00 / "piloto_fase5" / "repetibilidad_piloto.json"))
    ap.add_argument("--recalcular", action="store_true",
                    help="recalcula metricas.json de cada corrida en vez de leerlo")
    ap.add_argument("--repeticiones-como-sesiones", action="store_true",
                    help="ensayo, trata cada repetición del piloto como una sesión, para ejercitar "
                         "el contraste estadístico sin datos de campaña")
    ap.add_argument("--salida", default=str(SALIDA))
    args = ap.parse_args()

    cfg = json.load(open(args.config, encoding="utf-8"))
    punto = cfg["metricas"]["punto_error_lateral"]
    campo = f"rmse_e_y_{punto}_m"
    umbral = cargar_umbrales(args.umbrales)
    crit = cfg["criterio_mapa"]
    cfg_cond = {"max_pct_tc_sobre_periodo": crit["factibilidad"]["max_pct_ciclos_sobre_periodo"],
                "max_vueltas_fallidas": crit["factibilidad"]["max_vueltas_fallidas_por_celda"],
                "banda_bajo_costo": crit["esfuerzo_reportado"]["banda_bajo_costo"],
                "banda_actividad_superior": crit["esfuerzo_reportado"]["banda_actividad_superior"]}

    filas = recolectar(cfg, args.fase, set(args.perfiles), args.recalcular)
    if args.repeticiones_como_sesiones:
        # Ensayo sobre datos del piloto. El sufijo del identificador de plan
        # hace de sesión, solo para comprobar que el contraste corre.
        for f in filas:
            sufijo = f["id_plan"].rsplit("_", 1)[-1]
            f["sesion"] = int(sufijo) if sufijo.isdigit() else f["sesion"]
        print("ENSAYO, las repeticiones se tratan como sesiones. No es un resultado del estudio.")
    print(f"Fase {args.fase}, {len(filas)} corridas, punto de la métrica primaria {punto}")
    if not filas:
        print("Sin corridas para analizar.")
        return
    if umbral is None:
        print("AVISO, no hay umbrales del piloto, la mejora práctica no se evalúa")

    resultado = {"fase": args.fase, "punto_error_lateral": punto, "umbrales": umbral,
                 "condiciones_1_4": cfg_cond, "celdas": []}
    pvalores = {g: {} for g in GEOMETRICOS}
    for perfil in args.perfiles:
        for region in REGIONES:
            celda = analizar_celda(filas, region, perfil, campo, umbral, cfg_cond)
            resultado["celdas"].append(celda)
            for g, comp in celda["comparaciones"].items():
                if "wilcoxon_p" in comp:
                    pvalores[g][f"{perfil}|{region}"] = comp["wilcoxon_p"]

    # Holm dentro de cada familia de 9 celdas, decisión 4.4.
    for g in GEOMETRICOS:
        ajustados = holm(pvalores[g])
        for celda in resultado["celdas"]:
            clave = f"{celda['perfil']}|{celda['region']}"
            if g in celda["comparaciones"] and clave in ajustados:
                celda["comparaciones"][g]["p_holm"] = ajustados[clave]

    print(f"\n{'celda':22s} {'n':>3s} {'MPC':>8s} {'Stanley':>8s} {'PurePur':>8s}  "
          f"{'reduccion vs PP':>16s} {'reduccion vs St':>16s}")
    for celda in resultado["celdas"]:
        pc = celda["por_controlador"]
        if not pc:
            print(f"{celda['perfil']+' '+celda['region']:22s} sin datos")
            continue
        def r(c):
            return f"{pc[c]['rmse_medio_m']:.3f}" if c in pc else "  -  "
        def d(g):
            comp = celda["comparaciones"].get(g)
            if not comp:
                return "       -        "
            marca = ""
            if comp.get("supera_umbral") is True:
                marca = " *"
            elif comp.get("supera_umbral") is False:
                marca = " ."
            return f"{comp['reduccion_media_m']:+.3f}{marca:>3s}"
        print(f"{celda['perfil']+' '+celda['region']:22s} {celda['sesiones_completas']:3d} "
              f"{r(MPC):>8s} {r('stanley'):>8s} {r('pure_pursuit'):>8s}  "
              f"{d('pure_pursuit'):>16s} {d('stanley'):>16s}")

    print("\nLeyenda, * supera el umbral con el intervalo bootstrap por encima, . no lo supera.")
    print(f"\nEtiqueta por celda, bandas de esfuerzo {cfg_cond['banda_bajo_costo']:g} y "
          f"{cfg_cond['banda_actividad_superior']:g}")
    for celda in resultado["celdas"]:
        for g, comp in celda["comparaciones"].items():
            print(f"  {celda['perfil']:12s} {celda['region']:6s} contra {g:14s} razón de esfuerzo "
                  f"{comp['razon_esfuerzo_mpc_sobre_geometrico']:5.2f}  {comp['etiqueta']}")

    Path(args.salida).mkdir(parents=True, exist_ok=True)
    destino = Path(args.salida) / f"mapa_{args.fase}.json"
    json.dump(resultado, open(destino, "w", encoding="utf-8"), indent=2, ensure_ascii=False)
    print(f"\nGuardado en {destino}")


if __name__ == "__main__":
    main()
