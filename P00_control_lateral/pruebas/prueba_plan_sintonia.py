"""
Pruebas del plan de la fase 4. No tocan Assetto Corsa ni el plan real.

Se ejecutan con el repositorio como carpeta actual.
    python P00_control_lateral/pruebas/prueba_plan_sintonia.py
"""
import json
import sys
import tempfile
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from lanzador import plan_sintonia as ps  # noqa: E402

RESULTADOS = []


def verificar(nombre, ok, detalle=""):
    RESULTADOS.append((nombre, bool(ok)))
    print(f"[{'OK' if ok else 'FALLA'}] {nombre}" + (f" {detalle}" if detalle else ""))


RANGOS = {
    "sesion": 0,
    "controladores": {
        "pure_pursuit": {"L0_m": {"min": 1.5, "max": 5.0}, "kv_s": {"min": 0.3, "max": 0.9}},
        "stanley": {"k": {"min": 0.3, "max": 3.0}},
        "mpc_cinematico": {"Qpsi": {"min": 10.0, "max": 200.0, "escala": "log"},
                           "Rdelta": {"min": 0.1, "max": 10.0, "escala": "log"},
                           "Rddelta": {"min": 1.0, "max": 100.0, "escala": "log"}},
    },
}


def main():
    plan = ps.generar_plan(7, RANGOS, n_por_controlador=5)
    verificar("genera 5 corridas por controlador", len(plan["corridas"]) == 15, str(len(plan["corridas"])))
    verificar("el perfil es nominal en todas",
              all(c["perfil"] == "nominal" for c in plan["corridas"]))
    verificar("los identificadores no se repiten",
              len({c["id_plan"] for c in plan["corridas"]}) == 15)

    otra = ps.generar_plan(7, RANGOS, n_por_controlador=5)
    verificar("la misma semilla da el mismo plan",
              [c["parametros"] for c in plan["corridas"]] == [c["parametros"] for c in otra["corridas"]])
    distinta = ps.generar_plan(8, RANGOS, n_por_controlador=5)
    verificar("otra semilla da otro plan",
              [c["parametros"] for c in plan["corridas"]] != [c["parametros"] for c in distinta["corridas"]])

    dentro = True
    for c in plan["corridas"]:
        for nombre, valor in c["parametros"].items():
            r = RANGOS["controladores"][c["controlador"]][nombre]
            dentro = dentro and r["min"] <= valor <= r["max"]
    verificar("todos los valores caen dentro de su rango", dentro)

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        ruta = ps.guardar_plan_nuevo(plan, tmp / "plan_sintonia.json")
        verificar("guarda el plan", ruta.exists())
        try:
            ps.guardar_plan_nuevo(plan, ruta)
            verificar("no sobrescribe un plan existente", False)
        except FileExistsError:
            verificar("no sobrescribe un plan existente", True)

        rutas = ps.escribir_configs(plan, RAIZ_P00 / "configs" / "base.json", tmp / "sintonia")
        verificar("escribe una configuración por corrida", len(rutas) == 15)
        base = json.load(open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8"))
        corrida = plan["corridas"][0]
        cfg = json.load(open(ps.ruta_config(corrida, tmp / "sintonia"), encoding="utf-8"))
        for nombre, valor in corrida["parametros"].items():
            verificar(f"la configuración lleva {nombre} de la corrida",
                      cfg["controladores"][corrida["controlador"]][nombre] == valor)
        sin_controladores = {k: v for k, v in cfg.items() if k not in ("controladores", "_descripcion")}
        base_sin = {k: v for k, v in base.items() if k not in ("controladores", "_descripcion")}
        verificar("solo cambian los controladores y la descripción", sin_controladores == base_sin)
        otros = [n for n in ps.CONTROLADORES if n != corrida["controlador"]]
        verificar("los otros controladores quedan como la base",
                  all(cfg["controladores"][n] == base["controladores"][n] for n in otros))

    fila = ps.estado_plan(plan, [{"fase": "sintonia", "id_plan": plan["corridas"][0]["id_plan"]}])
    verificar("marca como hecha la corrida ya corrida", fila[0]["hecha"] and not fila[1]["hecha"])

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
    return 1 if fallas else 0


if __name__ == "__main__":
    sys.exit(main())
