"""
Plan de la campaña, decisión 4.2.

Cada una de las 10 sesiones corre los 3 controladores con los 3 perfiles en
orden aleatorio. El orden se genera una sola vez con una semilla, se guarda en
configs/plan_campana.json antes de ejecutar y nunca se sobrescribe.
"""
import json
import time
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
RUTA_PLAN = RAIZ_P00 / "configs" / "plan_campana.json"

CONTROLADORES = ("pure_pursuit", "stanley", "mpc_cinematico")
PERFILES = ("conservador", "nominal", "rapido")


def generar_plan(semilla, n_sesiones=10):
    rng = np.random.default_rng(int(semilla))
    combinaciones = [(c, p) for c in CONTROLADORES for p in PERFILES]
    sesiones = []
    for s in range(1, n_sesiones + 1):
        orden = rng.permutation(len(combinaciones))
        sesiones.append({
            "sesion": s,
            "corridas": [
                {"id_plan": f"s{s:02d}_p{k + 1}", "sesion": s, "posicion": k + 1,
                 "controlador": combinaciones[i][0], "perfil": combinaciones[i][1]}
                for k, i in enumerate(orden)
            ],
        })
    return {
        "semilla": int(semilla),
        "n_sesiones": n_sesiones,
        "generado": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "metodo": "numpy.random.default_rng(semilla).permutation de las 9 combinaciones en cada sesion",
        "numpy": np.__version__,
        "sesiones": sesiones,
    }


def cargar_plan(ruta=RUTA_PLAN):
    ruta = Path(ruta)
    if not ruta.exists():
        return None
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def guardar_plan_nuevo(plan, ruta=RUTA_PLAN):
    ruta = Path(ruta)
    if ruta.exists():
        raise FileExistsError(f"ya existe un plan en {ruta} y no se sobrescribe")
    ruta.parent.mkdir(parents=True, exist_ok=True)
    with open(ruta, "x", encoding="utf-8") as f:
        json.dump(plan, f, indent=2, ensure_ascii=False)


def corridas_en_orden(plan):
    return [c for s in plan["sesiones"] for c in s["corridas"]]
