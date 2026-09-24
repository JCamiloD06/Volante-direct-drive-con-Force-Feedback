"""
Repetibilidad y tiempo límite a partir del piloto, fase 5.

Aplica las reglas escritas en configs/parametros_piloto.json y las decisiones
1.2 y 5.1, sin decidir nada aquí.

Repetibilidad. Desviación estándar muestral del RMSE de e_y entre vueltas con
la misma configuración, por controlador. Se reporta con su intervalo de
confianza del 95 por ciento, obtenido de la distribución chi cuadrado, porque
con muestras pequeñas la estimación es imprecisa y ese error se traslada al
umbral de mejora práctica.

Umbral de mejora práctica, decisión 1.2. Para cada comparación del MPC contra
un geométrico, el mayor entre el 10 por ciento del RMSE del geométrico y dos
veces la desviación de repetibilidad de ese geométrico.

Tiempo límite, decisión 5.1. Doble de la mediana del tiempo de vuelta del
perfil más lento, redondeado hacia arriba a múltiplo de 10 s.

Uso
    python analisis/repetibilidad_piloto.py
Escribe piloto_fase5/repetibilidad_piloto.json y una tabla por pantalla.
"""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import stats

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

CORRIDAS = RAIZ.parent / "data" / "raw" / "p00" / "corridas"
SALIDA = RAIZ / "piloto_fase5"
TS = 0.05
GEOMETRICOS = ("pure_pursuit", "stanley")


def metricas_vuelta(carpeta):
    man = json.load(open(carpeta / "manifiesto.json", encoding="utf-8"))
    res = man.get("resumen") or {}
    fila = {"carpeta": carpeta.name, "id_plan": man["meta"].get("id_plan"),
            "controlador": man["meta"].get("controlador"),
            "perfil": man["meta"].get("perfil"),
            "vuelta_completada": bool(res.get("vuelta_completada")),
            "tiempo_vuelta_s": res.get("tiempo_vuelta_s"),
            "ciclos_ruedas_fuera": res.get("ciclos_ruedas_fuera"),
            "notas": man.get("notas", [])}
    if not fila["vuelta_completada"]:
        return fila
    with open(carpeta / "telemetria.csv", encoding="utf-8") as f:
        a = np.array([[r[c] for c in ("ciclo", "e_y_cg_m", "delta_aplicado_rad")]
                      for r in csv.DictReader(f)], dtype=float)
    k = (a[:, 0] >= res["cruce_inicio"]["ciclo"]) & (a[:, 0] <= res["cruce_fin"]["ciclo"])
    ey, delta = a[k, 1], a[k, 2]
    fila["rmse_e_y_m"] = float(np.sqrt(np.mean(ey ** 2)))
    fila["max_abs_e_y_m"] = float(np.abs(ey).max())
    fila["rms_tasa_rad_s"] = float(np.sqrt(np.mean((np.diff(delta) / TS) ** 2)))
    return fila


def intervalo_sigma(s, n, conf=0.95):
    """Intervalo de confianza de sigma a partir de la muestral, chi cuadrado."""
    if n < 2:
        return None, None
    gl = n - 1
    alfa = 1.0 - conf
    bajo = s * math.sqrt(gl / stats.chi2.ppf(1 - alfa / 2, gl))
    alto = s * math.sqrt(gl / stats.chi2.ppf(alfa / 2, gl))
    return float(bajo), float(alto)


def main():
    SALIDA.mkdir(exist_ok=True)
    par = json.load(open(RAIZ / "configs" / "parametros_piloto.json", encoding="utf-8"))
    plan = json.load(open(RAIZ / "configs" / "plan_piloto.json", encoding="utf-8"))
    bloque = {c["id_plan"]: c["bloque"] for c in plan["corridas"]}

    # Una corrida repetida deja dos carpetas. Se usa el último intento, mismo
    # criterio que el plan de campaña, y se deja constancia de cuántos hubo.
    por_id = {}
    for carpeta in sorted(CORRIDAS.glob("piloto_*")):
        if not (carpeta / "manifiesto.json").exists():
            continue
        f = metricas_vuelta(carpeta)
        f["bloque"] = bloque.get(f["id_plan"], "sin plan")
        por_id.setdefault(f["id_plan"], []).append(f)
    filas = []
    for idp, intentos in sorted(por_id.items()):
        ultimo = sorted(intentos, key=lambda x: x["carpeta"])[-1]
        ultimo["intentos"] = len(intentos)
        if len(intentos) > 1:
            print(f"  {idp} tiene {len(intentos)} intentos, se usa {ultimo['carpeta']}")
        filas.append(ultimo)

    rep = [f for f in filas if f["bloque"] == "repetibilidad"]
    descartadas = [f for f in rep if not f["vuelta_completada"]]
    validas = [f for f in rep if f["vuelta_completada"]]

    print(f"Repetibilidad, {len(rep)} corridas del bloque, {len(validas)} válidas")
    for f in descartadas:
        print(f"  descartada {f['id_plan']}, {'; '.join(f['notas']) or 'vuelta no completada'}")

    resumen = {"por_controlador": {}, "descartadas": [f["id_plan"] for f in descartadas]}
    print(f"\n{'controlador':16s} {'n':>3s} {'RMSE medio':>11s} {'sigma':>8s} "
          f"{'IC95 de sigma':>20s} {'2 sigma':>8s} {'10% del RMSE':>13s}")
    for controlador in ("pure_pursuit", "stanley", "mpc_cinematico"):
        v = [f["rmse_e_y_m"] for f in validas if f["controlador"] == controlador]
        if len(v) < 2:
            continue
        n = len(v)
        media = float(np.mean(v))
        s = float(np.std(v, ddof=1))
        bajo, alto = intervalo_sigma(s, n)
        resumen["por_controlador"][controlador] = {
            "n": n, "rmse_medio_m": media, "rmse_min_m": float(min(v)), "rmse_max_m": float(max(v)),
            "sigma_m": s, "ic95_sigma_m": [bajo, alto], "dos_sigma_m": 2 * s,
            "diez_por_ciento_rmse_m": 0.10 * media,
            "rms_tasa_media_rad_s": float(np.mean([f["rms_tasa_rad_s"] for f in validas
                                                   if f["controlador"] == controlador])),
        }
        print(f"{controlador:16s} {n:3d} {media:11.4f} {s:8.4f} "
              f"{f'[{bajo:.4f}, {alto:.4f}]':>20s} {2*s:8.4f} {0.10*media:13.4f}")

    # Umbral de mejora práctica por comparación, decisión 1.2.
    print(f"\nUmbral de mejora práctica del MPC")
    resumen["umbrales"] = {}
    for g in GEOMETRICOS:
        d = resumen["por_controlador"].get(g)
        if not d:
            continue
        umbral = max(d["diez_por_ciento_rmse_m"], d["dos_sigma_m"])
        manda = "dos sigma" if d["dos_sigma_m"] >= d["diez_por_ciento_rmse_m"] else "diez por ciento"
        resumen["umbrales"][g] = {"umbral_m": umbral, "manda": manda}
        mpc = resumen["por_controlador"].get("mpc_cinematico")
        linea = f"  contra {g:14s} umbral {umbral:.4f} m, manda el {manda}"
        if mpc:
            dif = d["rmse_medio_m"] - mpc["rmse_medio_m"]
            linea += (f", diferencia observada en el piloto {dif:.4f} m, "
                      f"{'supera' if dif > umbral else 'no supera'} el umbral")
        print(linea)

    # Tiempo límite, decisión 5.1.
    por_perfil = {}
    for f in filas:
        if f["vuelta_completada"] and f["tiempo_vuelta_s"]:
            por_perfil.setdefault(f["perfil"], []).append(f["tiempo_vuelta_s"])
    print(f"\nTiempos de vuelta por perfil")
    for perfil, t in sorted(por_perfil.items()):
        print(f"  {perfil:10s} n {len(t):2d} mediana {np.median(t):7.2f} s  min {min(t):7.2f}  max {max(t):7.2f}")
    if por_perfil:
        perfil_lento = max(por_perfil, key=lambda p: np.median(por_perfil[p]))
        mediana = float(np.median(por_perfil[perfil_lento]))
        limite = math.ceil(2 * mediana / 10.0) * 10
        resumen["tiempo_limite"] = {"perfil_mas_lento": perfil_lento, "mediana_s": mediana,
                                    "limite_s": limite, "regla": par["tiempo_limite"]["_regla"]}
        print(f"  perfil más lento {perfil_lento}, mediana {mediana:.2f} s, "
              f"tiempo límite {limite} s")

    resumen["vueltas"] = filas
    json.dump(resumen, open(SALIDA / "repetibilidad_piloto.json", "w", encoding="utf-8"),
              indent=2, ensure_ascii=False, default=str)
    print(f"\nGuardado en {SALIDA / 'repetibilidad_piloto.json'}")


if __name__ == "__main__":
    main()
