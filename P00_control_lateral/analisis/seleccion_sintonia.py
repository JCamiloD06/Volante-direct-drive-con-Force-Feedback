"""
Selección de la fase 4 con el criterio fijado en configs/rangos_sintonia.json.

El criterio se escribió el 2026-09-18, antes de generar el plan y antes de
correr ninguna vuelta de sintonía. Este script solo lo aplica.

Filtros, en este orden. Vuelta completada, cero ciclos con ruedas fuera, razón
de esfuerzo de dirección no mayor al tope y percentil 95 de la tasa no mayor al
tope. Entre las que pasan gana el menor RMSE de e_y. Un empate dentro del ruido
entre vueltas se resuelve por menor esfuerzo.

La razón de esfuerzo es el RMS de la tasa de dirección aplicada dividido por el
RMS de la tasa del ángulo geométrico de la trazada, atan(L kappa) derivado con
la velocidad de la misma vuelta. Se calcula entre los dos cruces de meta.

Uso
    python analisis/seleccion_sintonia.py
Escribe sintonia_fase4/seleccion_sintonia.json y una tabla por pantalla.
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ))

CORRIDAS = RAIZ.parent / "data" / "raw" / "p00" / "corridas"
SALIDA = RAIZ / "sintonia_fase4"
TS = 0.05
# Ruido entre vueltas medido en la fase 3, decisión 1.2.
RUIDO_RMSE_M = 0.012

COLUMNAS = ["ciclo", "v_kmh", "e_y_cg_m", "delta_aplicado_rad", "kappa_tras", "ruedas_fuera", "tc_ms"]


def metricas(carpeta):
    man = json.load(open(carpeta / "manifiesto.json", encoding="utf-8"))
    res = man.get("resumen") or {}
    m = {"carpeta": carpeta.name, "id_plan": man["meta"].get("id_plan"),
         "controlador": man["meta"].get("controlador"),
         "parametros": man["meta"].get("parametros_controlador"),
         "vuelta_completada": bool(res.get("vuelta_completada")),
         "tiempo_vuelta_s": res.get("tiempo_vuelta_s"),
         "notas": man.get("notas", [])}
    if not m["vuelta_completada"]:
        return m
    with open(carpeta / "telemetria.csv", encoding="utf-8") as f:
        a = np.array([[r[c] for c in COLUMNAS] for r in csv.DictReader(f)], dtype=float)
    T = {c: a[:, i] for i, c in enumerate(COLUMNAS)}
    k = (T["ciclo"] >= res["cruce_inicio"]["ciclo"]) & (T["ciclo"] <= res["cruce_fin"]["ciclo"])
    L = float(man["config"]["vehiculo"].get("batalla_m", 2.633))
    ey, delta, kappa = T["e_y_cg_m"][k], T["delta_aplicado_rad"][k], T["kappa_tras"][k]
    tasa = np.diff(delta) / TS
    tasa_geom = np.diff(np.arctan(L * kappa)) / TS
    base = float(np.sqrt(np.mean(tasa_geom ** 2)))
    m.update({
        "rmse_e_y_m": float(np.sqrt(np.mean(ey ** 2))),
        "max_abs_e_y_m": float(np.abs(ey).max()),
        "rms_tasa_rad_s": float(np.sqrt(np.mean(tasa ** 2))),
        "rms_tasa_geometrica_rad_s": base,
        "razon_esfuerzo": float(np.sqrt(np.mean(tasa ** 2)) / base),
        "p95_tasa_rad_s": float(np.percentile(np.abs(tasa), 95)),
        "ciclos_ruedas_fuera": int((T["ruedas_fuera"][k] > 0).sum()),
        "tc_ms_p95": float(np.percentile(T["tc_ms"][k], 95)),
        "ciclos": int(k.sum()),
    })
    return m


def filtrar(m, sel):
    """Devuelve la lista de motivos por los que la vuelta queda descartada."""
    motivos = []
    if not m["vuelta_completada"]:
        motivos.append("vuelta no completada")
        return motivos
    if sel.get("descartar_ruedas_fuera", True) and m["ciclos_ruedas_fuera"] > 0:
        motivos.append(f"{m['ciclos_ruedas_fuera']} ciclos con ruedas fuera")
    if m["razon_esfuerzo"] > sel["razon_esfuerzo_max"]:
        motivos.append(f"razon de esfuerzo {m['razon_esfuerzo']:.2f} sobre el tope "
                       f"{sel['razon_esfuerzo_max']}")
    if m["p95_tasa_rad_s"] > sel["p95_tasa_max_rad_s"]:
        motivos.append(f"p95 de tasa {m['p95_tasa_rad_s']:.3f} sobre el tope "
                       f"{sel['p95_tasa_max_rad_s']}")
    return motivos


def main():
    SALIDA.mkdir(exist_ok=True)
    rangos = json.load(open(RAIZ / "configs" / "rangos_sintonia.json", encoding="utf-8"))
    plan = json.load(open(RAIZ / "configs" / "plan_sintonia.json", encoding="utf-8"))
    sel = rangos["seleccion"]
    por_id = {}
    for carpeta in sorted(CORRIDAS.glob("sintonia_*")):
        if not (carpeta / "manifiesto.json").exists():
            print(f"sin manifiesto, se ignora, {carpeta.name}")
            continue
        m = metricas(carpeta)
        if m["id_plan"]:
            por_id.setdefault(m["id_plan"], []).append(m)

    resultado = {"criterio": sel, "semilla_plan": plan["semilla"], "controladores": {}}
    for controlador in ("pure_pursuit", "stanley", "mpc_cinematico"):
        delplan = [c for c in plan["corridas"] if c["controlador"] == controlador]
        filas = []
        for c in delplan:
            intentos = sorted(por_id.get(c["id_plan"], []), key=lambda x: x["carpeta"])
            if not intentos:
                filas.append({"id_plan": c["id_plan"], "parametros": c["parametros"],
                              "estado": "sin corrida", "descartes": ["sin corrida"]})
                continue
            m = intentos[-1]
            m["descartes"] = filtrar(m, sel)
            m["estado"] = "candidata" if not m["descartes"] else "descartada"
            m["intentos"] = len(intentos)
            filas.append(m)
        candidatas = [f for f in filas if f["estado"] == "candidata"]
        ganadora = None
        if candidatas:
            mejor = min(candidatas, key=lambda f: f["rmse_e_y_m"])
            empate = [f for f in candidatas if f["rmse_e_y_m"] - mejor["rmse_e_y_m"] <= RUIDO_RMSE_M]
            ganadora = min(empate, key=lambda f: f["razon_esfuerzo"]) if len(empate) > 1 else mejor
            ganadora = dict(ganadora, empatadas=[f["id_plan"] for f in empate])
        resultado["controladores"][controlador] = {
            "corridas": filas, "n_candidatas": len(candidatas), "ganadora": ganadora}

        print(f"\n===== {controlador}, {len(candidatas)} candidatas de {len(filas)}")
        print(f"{'id_plan':26s} {'e_y RMSE':>9s} {'max':>6s} {'razon':>6s} {'p95':>6s} "
              f"{'tc p95':>7s}  parametros / descarte")
        for f in sorted(filas, key=lambda f: f.get("rmse_e_y_m", 9e9)):
            p = ", ".join(f"{k}={v:g}" if isinstance(v, (int, float)) else f"{k}={v}"
                          for k, v in (f.get("parametros") or {}).items()
                          if k not in ("epsilon_ms", "_nota"))
            if f["estado"] != "candidata":
                print(f"{f['id_plan']:26s} {'':>9s} {'':>6s} {'':>6s} {'':>6s} {'':>7s}  "
                      f"{p} | DESCARTADA, {'; '.join(f['descartes'])}")
            else:
                marca = " <<<" if ganadora and f["id_plan"] == ganadora["id_plan"] else ""
                print(f"{f['id_plan']:26s} {f['rmse_e_y_m']:9.3f} {f['max_abs_e_y_m']:6.2f} "
                      f"{f['razon_esfuerzo']:6.2f} {f['p95_tasa_rad_s']:6.3f} {f['tc_ms_p95']:7.3f}  {p}{marca}")
        if ganadora:
            print(f"  GANADORA {ganadora['id_plan']} con {ganadora['parametros']}, "
                  f"RMSE {ganadora['rmse_e_y_m']:.3f} m, razón {ganadora['razon_esfuerzo']:.2f}")
            # Aviso de borde de rango, para declararlo en el manuscrito.
            for nombre, valor in ganadora["parametros"].items():
                r = rangos["controladores"][controlador].get(nombre)
                if not r or not isinstance(valor, (int, float)):
                    continue
                ancho = r["max"] - r["min"]
                if valor - r["min"] < 0.1 * ancho or r["max"] - valor < 0.1 * ancho:
                    print(f"  AVISO, {nombre}={valor:g} queda en el 10 por ciento extremo del rango "
                          f"[{r['min']}, {r['max']}]. El óptimo puede estar fuera.")

    json.dump(resultado, open(SALIDA / "seleccion_sintonia.json", "w", encoding="utf-8"),
              indent=2, ensure_ascii=False)
    print(f"\nGuardado en {SALIDA / 'seleccion_sintonia.json'}")


if __name__ == "__main__":
    main()
