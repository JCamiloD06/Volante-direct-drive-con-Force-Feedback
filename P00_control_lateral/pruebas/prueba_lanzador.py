"""
Prueba de la lógica del lanzador sin Assetto Corsa.

Comprueba el plan de campaña, la lectura de corridas guardadas, el estado del
plan, el resumen, los puntos de verificación y que la ventana se construye.
Todo en carpetas temporales, sin tocar data ni configs.

Uso desde la raíz del repositorio.
    python P00_control_lateral/pruebas/prueba_lanzador.py
"""
import json
import math
import sys
import tempfile
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from lanzador import corridas as mod_corridas  # noqa: E402
from lanzador import plan as mod_plan  # noqa: E402
from plataforma.perfil import construir_perfil  # noqa: E402
from plataforma.registro import COLUMNAS, RegistroCorrida  # noqa: E402
from plataforma.trazada import Trazada  # noqa: E402
from plataforma import procedencia  # noqa: E402

RESULTADOS = []


def verificar(nombre, condicion, detalle=""):
    RESULTADOS.append((nombre, bool(condicion)))
    print(f"[{'OK' if condicion else 'FALLA'}] {nombre} {detalle}")


def corrida_sintetica(base, cfg, trazada, perfil, meta, completada=True, n=400):
    reg = RegistroCorrida(str(base), f"{meta['fase']}_{meta.get('id_plan') or 'x'}_{n}", cfg,
                          RAIZ_P00 / "configs" / "base.json", meta)
    for k in range(n):
        fila = {c: 0.0 for c in COLUMNAS}
        norm = 0.2 * math.sin(k / 20.0)
        fila.update({"ciclo": k, "t_s": k * 0.05, "periodo_ms": 50.0, "tc_ms": 1.0, "retardo_ms": 2.0,
                     "contactos_ok": True, "L_medida_m": 2.63, "L_usada_m": 2.63,
                     "desalineacion_ejes_rad": 0.001, "deriva_rad": 0.003, "v_kmh": 72.0,
                     "steer_norm": norm, "steer_angle_ac": 0.95 * norm,
                     "tasa_guinada_local": 20.0 * math.tan(0.43 * norm) / 2.63, "acc_g_y": 0.1,
                     "ruedas_fuera": 0, "sat_magnitud": False, "sat_tasa": False, "ctrl_respaldo": 0,
                     "fase_vuelta": "medida" if 100 <= k < 380 else ("salida" if k < 100 else "terminada")})
        reg.agregar(fila)
    resumen = {"vuelta_completada": completada, "tiempo_vuelta_s": 14.0,
               "cruce_inicio": {"diferencia_velocidad_kmh": 3.0, "max_abs_e_y_ventana_m": 0.4},
               "pct_contactos_ok": 100.0, "L_medida_mediana_m": 2.63, "tc_ms_media": 1.0, "tc_ms_p95": 1.0,
               "tc_ms_max": 1.0, "pct_tc_mayor_50ms": 0.0, "periodo_ms_media": 50.0, "retardo_ms_p95": 2.0,
               "pct_sat_magnitud": 0.0, "pct_sat_tasa": 0.0, "pct_respaldo_controlador": 0.0}
    return reg.escribir(trazada, perfil, resumen, {"carModel": "alfa_romeo_giulietta_qv", "track": "monza",
                                                   "acVersion": "1.16.4"})


def main():
    cfg = json.load(open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8"))

    p1 = mod_plan.generar_plan(123)
    p2 = mod_plan.generar_plan(123)
    verificar("plan reproducible con la misma semilla", p1["sesiones"] == p2["sesiones"])
    verificar("plan distinto con otra semilla", p1["sesiones"] != mod_plan.generar_plan(124)["sesiones"])
    combinaciones = {(c, p) for c in mod_plan.CONTROLADORES for p in mod_plan.PERFILES}
    verificar("cada sesion corre las 9 combinaciones una vez",
              all({(r["controlador"], r["perfil"]) for r in s["corridas"]} == combinaciones
                  and len(s["corridas"]) == 9 for s in p1["sesiones"]))
    verificar("plan de 90 corridas", len(mod_plan.corridas_en_orden(p1)) == 90)

    with tempfile.TemporaryDirectory() as tmp:
        ruta_plan = Path(tmp) / "plan.json"
        mod_plan.guardar_plan_nuevo(p1, ruta_plan)
        try:
            mod_plan.guardar_plan_nuevo(p2, ruta_plan)
            verificar("un plan existente no se sobrescribe", False)
        except FileExistsError:
            verificar("un plan existente no se sobrescribe", True)

        trazada = Trazada(procedencia.RAIZ_REPO / cfg["trazada"]["csv"], cfg["trazada"]["suavizado_curvatura_m"])
        perfil = construir_perfil(trazada, cfg["perfil"], "nominal")
        orden = mod_plan.corridas_en_orden(p1)
        base = Path(tmp) / "corridas"
        for i, (r, completa) in enumerate(((orden[0], True), (orden[1], False))):
            corrida_sintetica(base, cfg, trazada, perfil,
                              {"fase": "campana", "id_plan": r["id_plan"], "sesion": r["sesion"],
                               "controlador": r["controlador"], "perfil": r["perfil"], "fisica_ampliada": True},
                              completada=completa, n=400 + i)
        carpeta_verif = corrida_sintetica(base, cfg, trazada, perfil,
                                          {"fase": "verificacion", "id_plan": "", "sesion": 0,
                                           "controlador": "stanley", "perfil": "conservador",
                                           "fisica_ampliada": True}, n=500)
        lista = mod_corridas.escanear(base)
        verificar("escaneo encuentra 3 corridas", len(lista) == 3)
        filas, siguiente = mod_corridas.estado_plan(p1, lista)
        verificar("estado del plan marca completada, no completada y siguiente",
                  filas[0]["estado"] == "completada" and filas[1]["estado"] == "vuelta no completada"
                  and siguiente == orden[2]["id_plan"], f"siguiente={siguiente}")
        texto = mod_corridas.resumen_legible(carpeta_verif)
        verificar("resumen legible", "Vuelta completada sí" in texto)
        items = mod_corridas.verificacion_fase3(carpeta_verif, cfg)
        por_nombre = {it["nombre"]: it for it in items}
        for it in items:
            print(f"       {it['nombre']}: {it['valor']} | {it['criterio']} | {it['cumple']}")
        verificar("verificacion reidentifica la constante de direccion cerca de 0.43",
                  por_nombre["Constante de la cadena de dirección"]["cumple"] is True)
        verificar("verificacion de vehiculo y tiempo real",
                  por_nombre["Vehículo"]["cumple"] and por_nombre["Ciclos con tiempo de cómputo bajo 50 ms"]["cumple"])

    try:
        from lanzador.app import Lanzador
        ventana = Lanzador()
        ventana.withdraw()
        ventana.update_idletasks()
        comando = ventana.var_comando.get()
        pestanas = [ventana.nb_principal.tab(t, "text") for t in ventana.nb_principal.tabs()]
        internas = [ventana.nb.tab(t, "text") for t in ventana.nb.tabs()]
        from lanzador.app import SCRIPT_MPC, FASES_CON_PLAN
        # Las tres tandas encadenadas existen, sintonía, piloto y campaña.
        tandas = [hasattr(ventana, n) for n in ("iniciar_tanda", "iniciar_tanda_piloto",
                                                "iniciar_tanda_campana")]
        # La campaña corre solo la sesión con pendientes más baja.
        ventana.filas_plan = {
            "s01_p1": {"id_plan": "s01_p1", "sesion": 1, "posicion": 1, "controlador": "stanley",
                       "perfil": "nominal", "estado": "pendiente"},
            "s01_p2": {"id_plan": "s01_p2", "sesion": 1, "posicion": 2, "controlador": "mpc_cinematico",
                       "perfil": "rapido", "estado": "pendiente"},
            "s02_p1": {"id_plan": "s02_p1", "sesion": 2, "posicion": 1, "controlador": "pure_pursuit",
                       "perfil": "nominal", "estado": "pendiente"},
        }
        pend = [f for f in ventana.filas_plan.values() if f["estado"] == "pendiente"]
        sesion_menor = min(f["sesion"] for f in pend)
        de_la_sesion = sorted([f for f in pend if f["sesion"] == sesion_menor],
                              key=lambda f: f["posicion"])
        # Sin identificador de plan, campana y sintonia no dejan armar el comando.
        errores_plan = 0
        for fase in FASES_CON_PLAN:
            ventana.var_fase.set(fase)
            ventana.var_id_plan.set("")
            try:
                ventana._argumentos(validar=True)
            except ValueError:
                errores_plan += 1
        ventana.var_fase.set("verificacion")
        ventana.destroy()
        verificar("la ventana se construye y arma el comando", "--controlador" in comando, comando[:90])
        verificar("dos pestañas principales", pestanas == ["Prueba de controladores", "MPC completo"], str(pestanas))
        verificar("las cinco pestañas de P00 quedan dentro",
                  internas == ["Corrida", "Plan de campaña", "Tanda de sintonía", "Piloto",
                               "Verificación fase 3"],
                  str(internas))
        verificar("script del MPC completo encontrado", SCRIPT_MPC.exists(), str(SCRIPT_MPC))
        verificar("existen las tres tandas encadenadas", all(tandas), str(tandas))
        verificar("la tanda de campaña toma solo la sesión pendiente más baja",
                  [f["id_plan"] for f in de_la_sesion] == ["s01_p1", "s01_p2"],
                  str([f["id_plan"] for f in de_la_sesion]))
        verificar("las fases con plan exigen identificador",
                  errores_plan == len(FASES_CON_PLAN), f"{errores_plan} de {len(FASES_CON_PLAN)}")
    except Exception as e:
        verificar("la ventana se construye y arma el comando", False, f"{type(e).__name__}: {e}")

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
        sys.exit(1)


if __name__ == "__main__":
    main()
