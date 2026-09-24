"""
Pruebas del plan del piloto. No tocan Assetto Corsa ni el plan real.

    python P00_control_lateral/pruebas/prueba_plan_piloto.py
"""
import json
import sys
import tempfile
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from lanzador import plan_piloto as pp  # noqa: E402

RESULTADOS = []


def verificar(nombre, ok, detalle=""):
    RESULTADOS.append((nombre, bool(ok)))
    print(f"[{'OK' if ok else 'FALLA'}] {nombre}" + (f" {detalle}" if detalle else ""))


def resultado(completada=True, fuera=0):
    return {"vuelta_completada": completada, "ciclos_ruedas_fuera": fuera}


def main():
    par = pp.cargar_parametros()
    plan = pp.generar_plan(par)
    ajuste = [c for c in plan["corridas"] if c["bloque"] == "ajuste_perfil"]
    rep = [c for c in plan["corridas"] if c["bloque"] == "repetibilidad"]

    verificar("el ajuste corre los tres controladores en cada nivel",
              len(ajuste) == 3 * len(par["ajuste_perfil"]["niveles"]), str(len(ajuste)))
    verificar("el ajuste usa el perfil rápido",
              all(c["perfil"] == "rapido" for c in ajuste))
    esperadas = int(par["repetibilidad"]["repeticiones"]) * len(par["repetibilidad"]["controladores"])
    verificar("la repetibilidad son las vueltas declaradas por controlador",
              len(rep) == esperadas, f"{len(rep)} de {esperadas}")
    verificar("la repetibilidad usa el perfil nominal",
              all(c["perfil"] == "nominal" for c in rep))
    verificar("la repetibilidad nace sin agarre asignado",
              all(c["grip_usage_factor"] is None for c in rep))
    verificar("los identificadores no se repiten",
              len({c["id_plan"] for c in plan["corridas"]}) == len(plan["corridas"]))

    # Criterio de aceptación del nivel.
    r = {}
    verificar("sin resultados no hay nivel aceptado", pp.nivel_aceptado(plan, r) is None)
    n0 = par["ajuste_perfil"]["niveles"][0]
    for c in ajuste:
        if c["nivel"] == n0:
            r[c["id_plan"]] = resultado(completada=(c["controlador"] != "pure_pursuit"))
    verificar("un nivel no se acepta si un controlador no completa",
              pp.nivel_aceptado(plan, r) is None)
    for c in ajuste:
        if c["nivel"] == n0 and c["controlador"] == "pure_pursuit":
            r[c["id_plan"]] = resultado(fuera=4)
    verificar("un nivel no se acepta con ciclos de ruedas fuera",
              pp.nivel_aceptado(plan, r) is None)
    for c in ajuste:
        if c["nivel"] == n0:
            r[c["id_plan"]] = resultado()
    verificar("un nivel se acepta con los tres limpios",
              pp.nivel_aceptado(plan, r) == n0, str(pp.nivel_aceptado(plan, r)))

    # Se prefiere el nivel más alto que pase.
    n1 = par["ajuste_perfil"]["niveles"][1]
    for c in ajuste:
        if c["nivel"] == n1:
            r[c["id_plan"]] = resultado()
    verificar("se queda con el nivel de agarre más alto que pasa",
              pp.nivel_aceptado(plan, r) == n0, str(pp.nivel_aceptado(plan, r)))

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        ruta = pp.guardar_plan_nuevo(plan, tmp / "plan_piloto.json")
        verificar("guarda el plan", ruta.exists())
        try:
            pp.guardar_plan_nuevo(plan, ruta)
            verificar("no sobrescribe un plan existente", False)
        except FileExistsError:
            verificar("no sobrescribe un plan existente", True)

        corrida = ajuste[0]
        destino = pp.escribir_config(corrida, RAIZ_P00 / "configs" / "base.json", tmp / "piloto")
        cfg = json.load(open(destino, encoding="utf-8"))
        base = json.load(open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8"))
        verificar("la configuración lleva el agarre del nivel",
                  cfg["perfil"]["grip_usage_factor"] == corrida["nivel"],
                  str(cfg["perfil"]["grip_usage_factor"]))
        verificar("los controladores quedan como en base.json",
                  cfg["controladores"] == base["controladores"])
        resto = {k: v for k, v in cfg.items() if k not in ("perfil", "_descripcion")}
        base_resto = {k: v for k, v in base.items() if k not in ("perfil", "_descripcion")}
        verificar("no cambia nada más que el perfil y la descripción", resto == base_resto)
        perfil_resto = {k: v for k, v in cfg["perfil"].items() if k != "grip_usage_factor"}
        base_perfil = {k: v for k, v in base["perfil"].items() if k != "grip_usage_factor"}
        verificar("dentro del perfil solo cambia el agarre", perfil_resto == base_perfil)

        try:
            pp.escribir_config(rep[0], RAIZ_P00 / "configs" / "base.json", tmp / "piloto")
            verificar("no escribe configuración sin agarre asignado", False)
        except ValueError:
            verificar("no escribe configuración sin agarre asignado", True)

        pp.asignar_agarre_repetibilidad(plan, n0, ruta)
        plan2 = pp.cargar_plan(ruta)
        rep2 = [c for c in plan2["corridas"] if c["bloque"] == "repetibilidad"]
        verificar("al fijar el agarre la repetibilidad lo recibe",
                  all(c["grip_usage_factor"] == n0 for c in rep2))
        verificar("queda registrado cuándo se fijó", "agarre_fijado" in plan2)
        try:
            pp.asignar_agarre_repetibilidad(plan2, n1, ruta)
            verificar("el agarre solo se fija una vez", False)
        except ValueError:
            verificar("el agarre solo se fija una vez", True)

        # Bloque de tiempo límite, añadido después de fijar el agarre.
        plan3 = pp.cargar_plan(ruta)
        plan3, nuevas = pp.anadir_bloque_tiempo_limite(plan3, ruta)
        tl = par["tiempo_limite"]
        verificar("el bloque de tiempo límite añade una vuelta por controlador",
                  len(nuevas) == len(tl["controladores"]), str(len(nuevas)))
        verificar("el bloque de tiempo límite usa el perfil más lento declarado",
                  all(c["perfil"] == tl["perfil"] for c in nuevas))
        verificar("hereda el agarre ya fijado",
                  all(c["grip_usage_factor"] == n0 for c in nuevas))
        try:
            pp.anadir_bloque_tiempo_limite(plan3, ruta)
            verificar("el bloque de tiempo límite solo se añade una vez", False)
        except ValueError:
            verificar("el bloque de tiempo límite solo se añade una vez", True)
        sin_fijar = pp.generar_plan(par)
        try:
            pp.anadir_bloque_tiempo_limite(sin_fijar, tmp / "otro.json")
            verificar("no se añade antes de fijar el agarre", False)
        except ValueError:
            verificar("no se añade antes de fijar el agarre", True)

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
