"""
Plan de la fase 4, sintonía de los tres controladores laterales, decisión 6.

Búsqueda aleatoria con semilla. Los rangos de cada parámetro se escriben en
`configs/rangos_sintonia.json` antes de generar el plan, y el plan se guarda en
`configs/plan_sintonia.json` y nunca se sobrescribe. Cada corrida del plan
lleva su propio archivo de configuración en `configs/sintonia/`, copia de la
configuración base con los parámetros del controlador sustituidos, para que el
manifiesto registre su huella sin que haya que tocar la base.

Los pesos del MPC se muestrean en escala logarítmica, porque lo que decide su
comportamiento son las razones entre ellos y no sus valores absolutos. Qy queda
fijo en el valor de la configuración base, ya que multiplicar los cuatro pesos
por una constante no cambia el óptimo del QP.
"""
import json
import time
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
RUTA_RANGOS = RAIZ_P00 / "configs" / "rangos_sintonia.json"
RUTA_PLAN = RAIZ_P00 / "configs" / "plan_sintonia.json"
DIR_CONFIGS = RAIZ_P00 / "configs" / "sintonia"

CONTROLADORES = ("pure_pursuit", "stanley", "mpc_cinematico")
PERFIL = "nominal"


def cargar_rangos(ruta=RUTA_RANGOS):
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def muestrear(rng, rango):
    """
    Un valor dentro del rango. Escala uniforme o logarítmica según se declare.
    El rango es {"min": a, "max": b, "escala": "lineal" o "log"}.
    """
    a, b = float(rango["min"]), float(rango["max"])
    if rango.get("escala", "lineal") == "log":
        return float(np.exp(rng.uniform(np.log(a), np.log(b))))
    return float(rng.uniform(a, b))


def generar_plan(semilla, rangos, n_por_controlador=20):
    rng = np.random.default_rng(int(semilla))
    corridas = []
    for controlador in CONTROLADORES:
        params_rango = rangos["controladores"][controlador]
        for k in range(1, n_por_controlador + 1):
            corridas.append({
                "id_plan": f"sint_{controlador}_{k:02d}",
                "controlador": controlador,
                "perfil": PERFIL,
                "sesion": int(rangos.get("sesion", 0)),
                "parametros": {n: round(muestrear(rng, r), 6) for n, r in params_rango.items()},
            })
    return {
        "semilla": int(semilla),
        "n_por_controlador": n_por_controlador,
        "perfil": PERFIL,
        "generado": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "metodo": ("numpy.random.default_rng(semilla).uniform sobre cada rango, en el orden "
                   "pure_pursuit, stanley y mpc_cinematico, y dentro de cada uno en el orden de los "
                   "parámetros del archivo de rangos"),
        "numpy": np.__version__,
        "rangos": rangos,
        "corridas": corridas,
    }


def guardar_plan_nuevo(plan, ruta=RUTA_PLAN):
    ruta = Path(ruta)
    if ruta.exists():
        raise FileExistsError(f"{ruta} ya existe. El plan de sintonía no se sobrescribe.")
    ruta.parent.mkdir(parents=True, exist_ok=True)
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(plan, f, ensure_ascii=False, indent=2)
    return ruta


def cargar_plan(ruta=RUTA_PLAN):
    ruta = Path(ruta)
    if not ruta.exists():
        return None
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def escribir_configs(plan, config_base, dir_configs=DIR_CONFIGS):
    """
    Un archivo de configuración por corrida del plan. Solo cambian los
    parámetros del controlador que se sintoniza. Devuelve la lista de rutas.
    """
    with open(config_base, encoding="utf-8") as f:
        base = json.load(f)
    dir_configs = Path(dir_configs)
    dir_configs.mkdir(parents=True, exist_ok=True)
    rutas = []
    for corrida in plan["corridas"]:
        cfg = json.loads(json.dumps(base))
        cfg["_descripcion"] = (f"Fase 4, corrida {corrida['id_plan']} del plan de sintonía con semilla "
                               f"{plan['semilla']}. Generada por lanzador/plan_sintonia.py. "
                               "NO es configuración de campaña.")
        cfg["controladores"][corrida["controlador"]].update(corrida["parametros"])
        ruta = dir_configs / f"{corrida['id_plan']}.json"
        with open(ruta, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        rutas.append(ruta)
    return rutas


def ruta_config(corrida, dir_configs=DIR_CONFIGS):
    return Path(dir_configs) / f"{corrida['id_plan']}.json"


def estado_plan(plan, corridas_guardadas):
    """
    Marca cada corrida del plan como hecha si existe una carpeta con fase
    sintonia y su id_plan. Mismo criterio que el plan de campaña, una corrida
    hecha no se repite aunque la vuelta haya fallado.
    """
    hechas = {c.get("id_plan") for c in corridas_guardadas if c.get("fase") == "sintonia"}
    filas = []
    for c in plan["corridas"]:
        filas.append(dict(c, hecha=c["id_plan"] in hechas))
    return filas
