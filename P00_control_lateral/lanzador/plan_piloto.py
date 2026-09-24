"""
Plan del piloto, fase 5.

Dos bloques que se corren en orden. El primero ajusta el perfil base, decisión
3.2, y el segundo mide la repetibilidad, decisión 1.2. El orden importa, porque
el agarre planificado entra en la construcción de los tres perfiles, de modo
que una repetibilidad medida antes del ajuste no describiría el perfil que se
usa en la campaña.

Bloque de ajuste. Escalera descendente de grip_usage_factor. Cada nivel corre
los tres controladores en perfil rápido. Un nivel se acepta solo si los tres
completan la vuelta sin ningún ciclo con ruedas fuera. La tanda se detiene en
el primer nivel aceptado, los niveles siguientes quedan sin correr.

Bloque de repetibilidad. Cinco vueltas por controlador en perfil nominal, con
el agarre ya fijado, todas con la misma configuración. De ahí sale la
desviación estándar del RMSE de e_y que entra en el umbral de mejora práctica.

Cada corrida lleva su archivo de configuración en configs/piloto/, copia de
base.json con grip_usage_factor sustituido. Los parámetros de los controladores
no se tocan, quedaron congelados al cerrar la fase 4.
"""
import json
import time
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
RUTA_PARAMETROS = RAIZ_P00 / "configs" / "parametros_piloto.json"
RUTA_PLAN = RAIZ_P00 / "configs" / "plan_piloto.json"
DIR_CONFIGS = RAIZ_P00 / "configs" / "piloto"

CONTROLADORES = ("pure_pursuit", "stanley", "mpc_cinematico")


def cargar_parametros(ruta=RUTA_PARAMETROS):
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def generar_plan(parametros):
    """
    Plan completo del piloto. El bloque de ajuste se genera entero, con todos
    los niveles, aunque la tanda se detenga en el primero que pase. El bloque
    de repetibilidad queda sin agarre asignado hasta que el ajuste termine.
    """
    aj = parametros["ajuste_perfil"]
    rep = parametros["repetibilidad"]
    sesion = int(parametros.get("sesion", 0))
    corridas = []
    for nivel in aj["niveles"]:
        etiqueta = f"{nivel:.2f}".replace("0.", "")
        for controlador in CONTROLADORES:
            corridas.append({
                "id_plan": f"pil_ajuste_{etiqueta}_{controlador}",
                "bloque": "ajuste_perfil",
                "nivel": float(nivel),
                "controlador": controlador,
                "perfil": aj["perfil"],
                "sesion": sesion,
                "grip_usage_factor": float(nivel),
            })
    for controlador in rep["controladores"]:
        for k in range(1, int(rep["repeticiones"]) + 1):
            corridas.append({
                "id_plan": f"pil_rep_{controlador}_{k:02d}",
                "bloque": "repetibilidad",
                "nivel": None,
                "controlador": controlador,
                "perfil": rep["perfil"],
                "sesion": sesion,
                "grip_usage_factor": None,
            })
    return {
        "generado": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "parametros": parametros,
        "corridas": corridas,
    }


def anadir_bloque_tiempo_limite(plan, ruta=RUTA_PLAN, parametros=None):
    """
    Añade al plan el bloque del tiempo límite, decisión 5.1, y lo reescribe.

    El plan original no incluía el perfil conservador, que es el más lento de
    los tres, de modo que la regla del tiempo límite no era aplicable con datos
    del piloto. El bloque se añade una sola vez, después de fijar el agarre,
    para que herede el mismo perfil que usará la campaña.

    Los parámetros del bloque se leen del archivo actual y no de la copia que
    el plan guardó al generarse, porque esa copia es anterior a la decisión de
    añadirlo. El plan registra la copia nueva, de modo que queda declarado con
    qué valores se creó el bloque.
    """
    if any(c["bloque"] == "tiempo_limite" for c in plan["corridas"]):
        raise ValueError("el bloque de tiempo límite ya existe en el plan")
    fijado = plan.get("agarre_fijado")
    if not fijado:
        raise ValueError("el agarre no está fijado, primero se cierra el ajuste del perfil")
    tl = (parametros or cargar_parametros())["tiempo_limite"]
    faltan = [k for k in ("perfil", "controladores") if k not in tl]
    if faltan:
        raise ValueError(f"parametros_piloto.json no declara {', '.join(faltan)} "
                         "en la sección tiempo_limite")
    plan["parametros"]["tiempo_limite"] = tl
    sesion = int(plan["parametros"].get("sesion", 0))
    nuevas = []
    for controlador in tl["controladores"]:
        for k in range(1, int(tl.get("repeticiones", 1)) + 1):
            nuevas.append({
                "id_plan": f"pil_tlim_{controlador}_{k:02d}",
                "bloque": "tiempo_limite",
                "nivel": None,
                "controlador": controlador,
                "perfil": tl["perfil"],
                "sesion": sesion,
                "grip_usage_factor": float(fijado["valor"]),
            })
    plan["corridas"].extend(nuevas)
    plan["bloque_tiempo_limite_anadido"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(plan, f, ensure_ascii=False, indent=2)
    return plan, nuevas


def guardar_plan_nuevo(plan, ruta=RUTA_PLAN):
    ruta = Path(ruta)
    if ruta.exists():
        raise FileExistsError(f"{ruta} ya existe. El plan del piloto no se sobrescribe.")
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


def escribir_config(corrida, config_base, dir_configs=DIR_CONFIGS):
    """
    Configuración de una corrida del piloto. Solo cambia grip_usage_factor y la
    descripción. Devuelve la ruta escrita.
    """
    with open(config_base, encoding="utf-8") as f:
        cfg = json.load(f)
    nivel = corrida["grip_usage_factor"]
    if nivel is None:
        raise ValueError(f"{corrida['id_plan']} no tiene agarre asignado, "
                         "el bloque de repetibilidad lo recibe al cerrar el ajuste")
    cfg["_descripcion"] = (f"Fase 5, corrida {corrida['id_plan']} del plan del piloto. "
                           f"grip_usage_factor {nivel:g}. Generada por lanzador/plan_piloto.py. "
                           "NO es configuración de campaña.")
    cfg["perfil"]["grip_usage_factor"] = float(nivel)
    dir_configs = Path(dir_configs)
    dir_configs.mkdir(parents=True, exist_ok=True)
    ruta = dir_configs / f"agarre_{nivel:.2f}.json".replace("0.", "")
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    return ruta


def ruta_config(corrida, dir_configs=DIR_CONFIGS):
    nivel = corrida["grip_usage_factor"]
    if nivel is None:
        return None
    return Path(dir_configs) / f"agarre_{nivel:.2f}.json".replace("0.", "")


def nivel_aceptado(plan, resultados):
    """
    Primer nivel de la escalera donde los tres controladores completaron la
    vuelta sin ciclos con ruedas fuera. Devuelve None si ninguno pasa todavía.

    resultados es un diccionario id_plan a dict con vuelta_completada y
    ciclos_ruedas_fuera.
    """
    crit = plan["parametros"]["ajuste_perfil"]["criterio_aceptacion"]
    maximo_fuera = int(crit.get("ciclos_ruedas_fuera_max", 0))
    por_nivel = {}
    for c in plan["corridas"]:
        if c["bloque"] == "ajuste_perfil":
            por_nivel.setdefault(c["nivel"], []).append(c)
    for nivel in sorted(por_nivel, reverse=True):
        filas = por_nivel[nivel]
        datos = [resultados.get(c["id_plan"]) for c in filas]
        if any(d is None for d in datos):
            continue
        if all(d.get("vuelta_completada") and d.get("ciclos_ruedas_fuera", 0) <= maximo_fuera
               for d in datos):
            return nivel
    return None


def asignar_agarre_repetibilidad(plan, nivel, ruta=RUTA_PLAN):
    """
    Fija el agarre del bloque de repetibilidad una vez aceptado un nivel, y
    reescribe el plan. Solo se permite una vez, para que el valor quede tan
    declarado como el resto del plan.
    """
    pendientes = [c for c in plan["corridas"]
                  if c["bloque"] == "repetibilidad" and c["grip_usage_factor"] is not None]
    if pendientes:
        raise ValueError("el bloque de repetibilidad ya tiene agarre asignado")
    for c in plan["corridas"]:
        if c["bloque"] == "repetibilidad":
            c["grip_usage_factor"] = float(nivel)
    plan["agarre_fijado"] = {"valor": float(nivel), "fecha": time.strftime("%Y-%m-%dT%H:%M:%S")}
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(plan, f, ensure_ascii=False, indent=2)
    return plan
