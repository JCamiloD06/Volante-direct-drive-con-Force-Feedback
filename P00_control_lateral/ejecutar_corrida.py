"""
Una corrida de P00 en Assetto Corsa con un controlador lateral y un perfil.

El bucle espera un paquete gráfico nuevo, lee el estado, calcula el comando
longitudinal común y el lateral del controlador elegido, retiene la dirección
si el vehículo está casi detenido, aplica los límites comunes, envía a vJoy
apenas termina el cálculo, decisión I6, y duerme lo que falte para completar
el periodo nominal. La corrida termina al completar la vuelta medida, decisión
I7, con Ctrl+C, al aparecer el archivo de detención que crea el lanzador o al
superar un tiempo máximo opcional. El registro se escribe siempre al salir.

Uso desde la raíz del repositorio, con Assetto Corsa abierto y el vehículo en pista.
    python P00_control_lateral/ejecutar_corrida.py --controlador stanley --perfil conservador --sesion 0 --fase verificacion
"""
import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

RAIZ_P00 = Path(__file__).resolve().parent
sys.path.insert(0, str(RAIZ_P00))

import numpy as np  # noqa: E402

from controladores import NOMBRES, crear_controlador  # noqa: E402
from controladores.base import EntradaLateral  # noqa: E402
from plataforma import procedencia  # noqa: E402
from plataforma.abandono import Abandono  # noqa: E402
from plataforma.actuador import InterfazDireccion, SalidaVJoy, entrada_progresiva  # noqa: E402
from plataforma.angulos import envolver  # noqa: E402
from plataforma.longitudinal import PedalShaper, PIDLongitudinalController  # noqa: E402
from plataforma.memoria_ac import MemoriaAC, construir_estado  # noqa: E402
from plataforma.perfil import construir_perfil  # noqa: E402
from plataforma.registro import RegistroCorrida  # noqa: E402
from plataforma.trazada import Proyector, Trazada  # noqa: E402
from plataforma.vuelta import SeguidorVuelta  # noqa: E402

PERFILES = ("conservador", "nominal", "rapido")
FASES = ("verificacion", "sintonia", "piloto", "campana", "prueba")
PREFIJO_GUARDADA = "CORRIDA_GUARDADA|"


def _resumen_ciclos(filas):
    if not filas:
        return {}
    tc = np.array([f["tc_ms"] for f in filas], dtype=float)
    per = np.array([f["periodo_ms"] for f in filas], dtype=float)
    ret = np.array([f["retardo_ms"] for f in filas], dtype=float)
    L = np.array([f["L_medida_m"] for f in filas], dtype=float)
    return {
        "tc_ms_media": float(np.nanmean(tc)),
        "tc_ms_p95": float(np.nanpercentile(tc, 95)),
        "tc_ms_max": float(np.nanmax(tc)),
        "pct_tc_mayor_50ms": float(np.mean(tc > 50.0) * 100.0),
        "periodo_ms_media": float(np.nanmean(per)) if np.isfinite(per).any() else None,
        "retardo_ms_p95": float(np.nanpercentile(ret, 95)),
        "pct_contactos_ok": float(np.mean([f["contactos_ok"] for f in filas]) * 100.0),
        "L_medida_mediana_m": float(np.nanmedian(L)) if np.isfinite(L).any() else None,
        "pct_sat_magnitud": float(np.mean([f["sat_magnitud"] for f in filas]) * 100.0),
        "pct_sat_tasa": float(np.mean([f["sat_tasa"] for f in filas]) * 100.0),
        "pct_direccion_retenida": float(np.mean([f["direccion_retenida"] for f in filas]) * 100.0),
        "pct_respaldo_controlador": float(np.mean([f["ctrl_respaldo"] for f in filas]) * 100.0),
        # Ruedas fuera sobre los ciclos registrados. Entra en el criterio de
        # aceptacion del perfil base del piloto y en la decision 5.1.
        "ciclos_ruedas_fuera": int(sum(1 for f in filas if f["ruedas_fuera"] > 0)),
        "max_ruedas_fuera": int(max(f["ruedas_fuera"] for f in filas)),
    }


def main():
    ap = argparse.ArgumentParser(description="Corrida de P00 en Assetto Corsa")
    ap.add_argument("--controlador", required=True, choices=NOMBRES)
    ap.add_argument("--perfil", required=True, choices=PERFILES)
    ap.add_argument("--sesion", required=True, type=int)
    ap.add_argument("--fase", default="prueba", choices=FASES)
    ap.add_argument("--id-plan", default="", help="obligatorio en fase campana")
    ap.add_argument("--config", default=str(RAIZ_P00 / "configs" / "base.json"))
    ap.add_argument("--etiqueta", default="")
    ap.add_argument("--tiempo-max-s", type=float, default=None,
                    help="detiene la corrida si se supera este tiempo")
    ap.add_argument("--archivo-detener", default=None,
                    help="si este archivo aparece, la corrida se detiene y guarda el registro")
    ap.add_argument("--sin-verificar-vehiculo", action="store_true",
                    help="solo para pruebas, no verifica el modelo de vehículo")
    ap.add_argument("--info-sesion-ac", default=None,
                    help="JSON que deja el lanzador al preparar Assetto Corsa, se copia al manifiesto")
    args = ap.parse_args()

    sesion_ac = None
    if args.info_sesion_ac:
        with open(args.info_sesion_ac, encoding="utf-8") as f:
            sesion_ac = json.load(f)

    if args.fase == "campana" and not args.id_plan:
        print("Una corrida de campaña debe indicar --id-plan. Corrida cancelada.")
        sys.exit(2)

    detener = {"senal": False}
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, lambda *_: detener.__setitem__("senal", True))

    ruta_config = Path(args.config)
    with open(ruta_config, encoding="utf-8") as f:
        cfg = json.load(f)
    ts = float(cfg["ts_s"])

    origen = procedencia.estado_origen()
    if not (origen["barrido_sin_cambios"] and origen["trazada_sin_cambios"]):
        print("AVISO. Un archivo de origen cambió desde que se copió. Queda registrado en el manifiesto.")

    trazada = Trazada(procedencia.RAIZ_REPO / cfg["trazada"]["csv"],
                      cfg["trazada"]["suavizado_curvatura_m"])
    perfil = construir_perfil(trazada, cfg["perfil"], args.perfil)
    controlador = crear_controlador(args.controlador, cfg)

    mem = MemoriaAC()
    info_estatica = mem.leer_estatica()
    esperado = cfg["vehiculo"]["modelo_esperado"]
    if info_estatica["carModel"] != esperado and not args.sin_verificar_vehiculo:
        print(f"Vehículo leído {info_estatica['carModel']} distinto del esperado {esperado}. Corrida cancelada.")
        mem.cerrar()
        sys.exit(2)

    d = cfg["direccion"]
    interfaz = InterfazDireccion(d["delta_max_rad"], d["tasa_max_rad_s"], ts)
    v_nula = float(d["velocidad_minima_direccion_kmh"])
    v_plena = float(d["velocidad_plena_direccion_kmh"])
    salida = SalidaVJoy(d["rueda_rad_eje_completo"], d["signo_vjoy"])
    lo = cfg["longitudinal"]
    pid = PIDLongitudinalController(lo["kp"], lo["ki"], lo["kd"], dt=ts,
                                    limite_integral=lo["limite_integral"])
    pedal = PedalShaper(ts, **lo["pedal"])
    cv = cfg["vuelta"]
    vuelta = SeguidorVuelta(cv["ventana_estabilizacion_s"], cv["pos_cruce_alta"], cv["pos_cruce_baja"])
    tiempo_max = args.tiempo_max_s if args.tiempo_max_s is not None else cv.get("tiempo_limite_s")
    proy_cg, proy_tras, proy_del = Proyector(trazada), Proyector(trazada), Proyector(trazada)

    id_corrida = (f"{args.fase}_s{args.sesion:02d}_{args.controlador}_{args.perfil}_"
                  f"{time.strftime('%Y%m%d_%H%M%S')}")
    registro = RegistroCorrida(
        str(procedencia.RAIZ_REPO / cfg["salida"]["directorio_corridas"]), id_corrida, cfg, ruta_config,
        meta={
            "controlador": args.controlador,
            "parametros_controlador": controlador.parametros,
            "perfil": args.perfil,
            "factor_perfil": perfil.factor,
            "sesion": args.sesion,
            "fase": args.fase,
            "id_plan": args.id_plan,
            "etiqueta": args.etiqueta,
            "fisica_ampliada": mem.fisica_ampliada,
            "motivo_sin_fisica_ampliada": mem.motivo_sin_ampliada,
            "sin_verificar_vehiculo": args.sin_verificar_vehiculo,
            "sesion_ac": sesion_ac,
            "argv": sys.argv,
        })

    print("=" * 70)
    print(f"  P00 corrida {id_corrida}")
    print(f"  vehículo {info_estatica['carModel']}  pista {info_estatica['track']}  AC {info_estatica['acVersion']}")
    print(f"  física ampliada {mem.fisica_ampliada}  perfil f={perfil.factor}  fase {args.fase}  plan {args.id_plan}")
    print("=" * 70)

    ciclo = 0
    ultimo_packet = None
    repetidos = 0
    vueltas_ac = 0
    t0 = time.perf_counter()
    t_prev = None
    vigilante = Abandono(cfg.get("abandono"))
    try:
        while True:
            gr = mem.leer_graficos()
            if gr.packetId == ultimo_packet:
                repetidos += 1
                # La señal de parada se revisa también aquí. Si el juego pausa
                # o congela, el paquete deja de cambiar y sin esto la corrida
                # no responde al lanzador, verificado el 2026-09-18.
                if detener["senal"] or (args.archivo_detener and repetidos % 200 == 0
                                        and os.path.exists(args.archivo_detener)):
                    registro.nota("detenida desde el lanzador sin paquete nuevo del juego")
                    print("Corrida detenida desde el lanzador.")
                    break
                time.sleep(0.002)
                continue
            ultimo_packet = gr.packetId
            t_lectura = time.perf_counter()
            ph = mem.leer_fisica()
            periodo_ms = float("nan") if t_prev is None else (t_lectura - t_prev) * 1000.0
            t_prev = t_lectura

            est = construir_estado(gr, ph, cfg["vehiculo"], t_lectura, mem.fisica_ampliada)
            vueltas_ac = est.vueltas_completadas
            p_cg = proy_cg.proyectar(est.x, est.z)
            p_tr = proy_tras.proyectar(est.x_tras, est.z_tras)
            p_de = proy_del.proyectar(est.x_del, est.z_del)
            e_psi_cg = envolver(est.psi - p_cg.phi)
            e_psi_tr = envolver(est.psi - p_tr.phi)
            e_psi_de = envolver(est.psi - p_de.phi)

            v_ref = perfil.objetivo_kmh(p_cg.idx, est.v_ms)
            u_pid = pid.compute_control(est.v_kmh, v_ref)
            u_pedal = pedal.shape(u_pid)

            entrada = EntradaLateral(
                trazada=trazada, v_ms=est.v_ms, L=est.L_usada, psi=est.psi,
                x_tras=est.x_tras, z_tras=est.z_tras, x_del=est.x_del, z_del=est.z_del,
                idx_tras=p_tr.idx, e_y_tras=p_tr.e_y, e_psi_tras=e_psi_tr,
                idx_del=p_de.idx, e_y_del=p_de.e_y, e_psi_del=e_psi_de,
                delta_prev=interfaz.delta_prev)
            t_c0 = time.perf_counter()
            sal = controlador.calcular(entrada)
            tc_ms = (time.perf_counter() - t_c0) * 1000.0

            delta_escalado, factor_dir = entrada_progresiva(sal.delta, est.v_kmh, v_nula, v_plena)
            retenida = factor_dir == 0.0
            delta_aplicado, sat_mag, sat_tasa = interfaz.aplicar(delta_escalado, time.perf_counter())
            gas, freno, norm, sat_eje = salida.enviar(delta_aplicado, u_pedal)
            retardo_ms = (time.perf_counter() - t_lectura) * 1000.0

            fase = vuelta.actualizar(ciclo, t_lectura - t0, est.vueltas_completadas,
                                     est.posicion_normalizada, est.v_kmh, v_ref, p_cg.e_y)
            deriva = (envolver(est.rumbo_velocidad - est.psi)
                      if math.isfinite(est.rumbo_velocidad) else float("nan"))

            registro.agregar({
                "ciclo": ciclo, "t_s": t_lectura - t0, "periodo_ms": periodo_ms,
                "tc_ms": tc_ms, "retardo_ms": retardo_ms,
                "packet_graficos": est.packet_graficos, "packet_fisica": est.packet_fisica,
                "packets_repetidos": repetidos,
                "x": est.x, "y": est.y, "z": est.z, "psi_rad": est.psi,
                "heading_ac_rad": est.heading_ac, "rumbo_velocidad_rad": est.rumbo_velocidad,
                "deriva_rad": deriva,
                "contactos_ok": est.contactos_ok, "motivo_contactos": est.motivo_contactos,
                "x_tras": est.x_tras, "z_tras": est.z_tras, "x_del": est.x_del, "z_del": est.z_del,
                "L_medida_m": est.L_medida, "L_usada_m": est.L_usada,
                "desalineacion_ejes_rad": est.desalineacion_ejes,
                "idx_cg": p_cg.idx, "s_cg_m": p_cg.s, "e_y_cg_m": p_cg.e_y,
                "e_psi_cg_rad": e_psi_cg, "kappa_cg": p_cg.kappa,
                "idx_tras": p_tr.idx, "s_tras_m": p_tr.s, "e_y_tras_m": p_tr.e_y,
                "e_psi_tras_rad": e_psi_tr, "kappa_tras": p_tr.kappa,
                "idx_del": p_de.idx, "s_del_m": p_de.s, "e_y_del_m": p_de.e_y,
                "e_psi_del_rad": e_psi_de,
                "v_kmh": est.v_kmh, "v_ref_kmh": v_ref,
                "v_perfil_kmh": perfil.velocidad_perfil_kmh(p_cg.idx),
                "delta_pedido_rad": sal.delta, "delta_aplicado_rad": delta_aplicado,
                "factor_direccion": factor_dir, "direccion_retenida": retenida,
                "sat_magnitud": sat_mag, "sat_tasa": sat_tasa,
                "steer_norm": norm, "sat_eje": sat_eje, "steer_angle_ac": est.steer_angle_ac,
                "ctrl_estado": sal.estado, "ctrl_iteraciones": sal.iteraciones,
                "ctrl_respaldo": sal.respaldo, "ctrl_extra1": sal.extra1, "ctrl_extra2": sal.extra2,
                "u_pid": u_pid, "u_pedal": u_pedal, "gas_cmd": gas, "freno_cmd": freno,
                "gas_ac": est.gas_ac, "freno_ac": est.freno_ac,
                "acc_g_x": est.acc_g_x, "acc_g_y": est.acc_g_y, "acc_g_z": est.acc_g_z,
                "tasa_guinada_local": est.tasa_guinada_local,
                "ruedas_fuera": est.ruedas_fuera, "vueltas_completadas": est.vueltas_completadas,
                "posicion_normalizada": est.posicion_normalizada, "en_pits": est.en_pits,
                "fase_vuelta": fase,
            })
            repetidos = 0
            ciclo += 1

            if ciclo % 20 == 0:
                marca = "RET" if retenida else "   "
                print(f"{t_lectura - t0:7.1f}s {fase:9s} e_y={p_cg.e_y:+6.2f} m "
                      f"δ={math.degrees(delta_aplicado):+6.2f}° {marca} v={est.v_kmh:6.1f}/{v_ref:6.1f} km/h "
                      f"tc={tc_ms:5.1f} ms pos={est.posicion_normalizada:.3f} "
                      f"contactos={'ok' if est.contactos_ok else est.motivo_contactos}")

            motivo_abandono = vigilante.actualizar(t_lectura, est.v_kmh, est.en_pits, est.ruedas_fuera)
            if motivo_abandono:
                registro.nota(f"abandono automatico, {motivo_abandono}")
                print(f"Abandono automático, {motivo_abandono}. La corrida se guarda.")
                break

            if fase == "terminada":
                print("Vuelta medida completada.")
                break
            if tiempo_max is not None and t_lectura - t0 > tiempo_max:
                registro.nota(f"detenida por tiempo maximo de {tiempo_max} s")
                print("Tiempo máximo superado, corrida detenida.")
                break
            if detener["senal"] or (args.archivo_detener and ciclo % 5 == 0
                                    and os.path.exists(args.archivo_detener)):
                registro.nota("detenida desde el lanzador")
                print("Corrida detenida desde el lanzador.")
                break

            espera = ts - (time.perf_counter() - t_lectura)
            if espera > 0:
                time.sleep(espera)
    except KeyboardInterrupt:
        registro.nota("interrumpida por el usuario")
        print("\nCorrida interrumpida por el usuario.")
    finally:
        salida.centrar()
        try:
            info_estatica = mem.leer_estatica()
        except Exception:
            pass
        mem.cerrar()
        if ciclo > 0:
            resumen = vuelta.resumen(vueltas_ac)
            resumen.update(_resumen_ciclos(registro.filas))
            carpeta = registro.escribir(trazada, perfil, resumen, info_estatica)
            print(f"Ciclos {ciclo}. Corrida guardada en {carpeta}")
            print(f"{PREFIJO_GUARDADA}{carpeta}")
        else:
            print("No se registró ningún ciclo, no se guarda la corrida.")


if __name__ == "__main__":
    main()
