"""
Prueba de humo de P00 sin Assetto Corsa.

Comprueba que los módulos cargan y que cada pieza hace lo que dicen sus
ecuaciones en casos con respuesta conocida. Es verificación de implementación
y no produce resultados del estudio.

Uso desde la raíz del repositorio.
    python P00_control_lateral/pruebas/prueba_humo.py
"""
import csv
import ctypes
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from controladores import crear_controlador  # noqa: E402
from controladores.base import EntradaLateral  # noqa: E402
from plataforma import procedencia  # noqa: E402
from plataforma.actuador import InterfazDireccion, SalidaSimulada, entrada_progresiva  # noqa: E402
from plataforma.memoria_ac import (SPageFileGraphic, SPageFilePhysics,  # noqa: E402
                                   SPageFilePhysicsAmpliada, construir_estado)
from plataforma.perfil import construir_perfil  # noqa: E402
from plataforma.registro import COLUMNAS, RegistroCorrida  # noqa: E402
from plataforma.trazada import Proyector, Trazada  # noqa: E402
from plataforma.vuelta import SeguidorVuelta  # noqa: E402

RESULTADOS = []


def verificar(nombre, condicion, detalle=""):
    RESULTADOS.append((nombre, bool(condicion)))
    print(f"[{'OK' if condicion else 'FALLA'}] {nombre} {detalle}")


def entrada_en(trazada, idx_tras, L, v, psi=None, desplazamiento_izq=0.0, delta_prev=0.0):
    """Eje trasero sobre el vértice idx_tras, desplazado a la izquierda si se pide."""
    phi = float(trazada.tangente_vertice[idx_tras])
    psi = phi if psi is None else psi
    nx, nz = -math.sin(phi), math.cos(phi)
    xt = float(trazada.x[idx_tras]) + desplazamiento_izq * nx
    zt = float(trazada.z[idx_tras]) + desplazamiento_izq * nz
    xd, zd = xt + L * math.cos(psi), zt + L * math.sin(psi)
    pt = Proyector(trazada).proyectar(xt, zt)
    pd = Proyector(trazada).proyectar(xd, zd)
    return EntradaLateral(trazada=trazada, v_ms=v, L=L, psi=psi,
                          x_tras=xt, z_tras=zt, x_del=xd, z_del=zd,
                          idx_tras=pt.idx, e_y_tras=pt.e_y,
                          e_psi_tras=math.remainder(psi - pt.phi, 2 * math.pi),
                          idx_del=pd.idx, e_y_del=pd.e_y,
                          e_psi_del=math.remainder(psi - pd.phi, 2 * math.pi),
                          delta_prev=delta_prev)


def trazada_circulo(radio, espaciado, carpeta):
    n = int(round(2 * math.pi * radio / espaciado))
    ruta = Path(carpeta) / "circulo.csv"
    with open(ruta, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "x", "y_elevation", "z", "cumulative_length_m"])
        for i in range(n):
            th = 2 * math.pi * i / n
            w.writerow([i, radio * math.cos(th), 0.0, radio * math.sin(th), radio * th])
    return Trazada(ruta, suavizado_curvatura_m=0.0)


def main():
    cfg = json.load(open(RAIZ_P00 / "configs" / "base.json", encoding="utf-8"))
    L = 2.633

    origen = procedencia.estado_origen()
    verificar("origen sin cambios", origen["barrido_sin_cambios"] and origen["trazada_sin_cambios"])

    monza = Trazada(procedencia.RAIZ_REPO / cfg["trazada"]["csv"], cfg["trazada"]["suavizado_curvatura_m"])
    i0 = 1000
    phi = float(monza.tangente_vertice[i0])
    p = Proyector(monza).proyectar(monza.x[i0] - math.sin(phi), monza.z[i0] + math.cos(phi))
    verificar("proyeccion 1 m a la izquierda da e_y +1", abs(p.e_y - 1.0) < 0.05, f"e_y={p.e_y:.4f}")
    for nombre in ("conservador", "nominal", "rapido"):
        perf = construir_perfil(monza, cfg["perfil"], nombre)
        print(f"       perfil {nombre} f={perf.factor} v min {perf.v_max.min() * 3.6:.1f} "
              f"max {perf.v_max.max() * 3.6:.1f} km/h")
    verificar("perfil escalado proporcional",
              abs(construir_perfil(monza, cfg["perfil"], "conservador").v_max.max()
                  / construir_perfil(monza, cfg["perfil"], "rapido").v_max.max() - 0.8) < 1e-9)

    pp = crear_controlador("pure_pursuit", cfg)
    st = crear_controlador("stanley", cfg)

    recta = next(i for i in range(monza.n)
                 if np.abs(monza.curvature[(i + np.arange(int(80 / monza.avg_spacing))) % monza.n]).max() < 2e-4)
    e0 = entrada_en(monza, recta, L, 30.0)
    verificar("PP en recta sin error da delta cero", abs(pp.calcular(e0).delta) < 0.005,
              f"delta={pp.calcular(e0).delta:.5f}")
    verificar("Stanley sin error da delta cero", abs(st.calcular(e0).delta) < 0.005,
              f"delta={st.calcular(e0).delta:.5f}")
    e_izq = entrada_en(monza, recta, L, 30.0, desplazamiento_izq=1.0)
    verificar("PP corrige hacia la derecha si esta a la izquierda", pp.calcular(e_izq).delta < 0)
    verificar("Stanley corrige hacia la derecha si esta a la izquierda", st.calcular(e_izq).delta < 0)

    with tempfile.TemporaryDirectory() as tmp:
        R = 50.0
        circ = trazada_circulo(R, 0.1, tmp)
        ec = entrada_en(circ, 100, L, 15.0, delta_prev=math.atan(L / R))
        esperado = math.atan(L / R)
        d_pp = pp.calcular(ec).delta
        verificar("PP en circulo da atan(L/R)", abs(d_pp / esperado - 1) < 0.02,
                  f"delta={d_pp:.5f} esperado={esperado:.5f}")
        mpc_c = crear_controlador("mpc_cinematico", cfg)
        d_mpc = mpc_c.calcular(ec).delta
        verificar("MPC en circulo sin error da el angulo de regimen", abs(d_mpc / esperado - 1) < 0.03,
                  f"delta={d_mpc:.5f} esperado={esperado:.5f}")

    # Arranque con el vehículo detenido. Antes del 2026-09-15 el MPC usaba una
    # velocidad mínima de 1 m/s y enrollaba la dirección hasta el tope.
    mpc_p = crear_controlador("mpc_cinematico", cfg)
    delta_parado = 0.0
    for _ in range(40):
        e_parado = entrada_en(monza, recta, L, 0.0, desplazamiento_izq=2.59, delta_prev=delta_parado)
        delta_parado = mpc_p.calcular(e_parado).delta
    verificar("MPC detenido no enrolla la direccion", abs(math.degrees(delta_parado)) < 1.0,
              f"delta tras 40 ciclos={math.degrees(delta_parado):.3f} grados")
    v_nula = cfg["direccion"]["velocidad_minima_direccion_kmh"]
    v_plena = cfg["direccion"]["velocidad_plena_direccion_kmh"]
    medio = 0.5 * (v_nula + v_plena)
    verificar("entrada progresiva de direccion",
              entrada_progresiva(0.3, v_nula - 1, v_nula, v_plena) == (0.0, 0.0)
              and entrada_progresiva(0.3, v_plena + 1, v_nula, v_plena) == (0.3, 1.0)
              and abs(entrada_progresiva(0.3, medio, v_nula, v_plena)[0] - 0.15) < 1e-12,
              f"rampa de {v_nula} a {v_plena} km/h")

    mpc = crear_controlador("mpc_cinematico", cfg)
    s_mpc = mpc.calcular(e_izq)
    verificar("MPC corrige hacia la derecha si esta a la izquierda", s_mpc.delta < 0,
              f"delta={s_mpc.delta:.5f} estado={s_mpc.estado}")
    rng = np.random.default_rng(20260915)
    tiempos, estados = [], []
    mpc.reiniciar()
    for _ in range(300):
        idx = int(rng.integers(0, monza.n))
        e = entrada_en(monza, idx, L, float(rng.uniform(15, 50)), desplazamiento_izq=float(rng.normal(0, 0.5)),
                       delta_prev=float(rng.uniform(-0.05, 0.05)))
        t0 = time.perf_counter()
        s = mpc.calcular(e)
        tiempos.append((time.perf_counter() - t0) * 1000)
        estados.append(s.estado)
    tiempos = np.array(tiempos)
    print(f"       MPC N={mpc.N} en este equipo, 300 estados aleatorios. mediana {np.median(tiempos):.2f} ms "
          f"p95 {np.percentile(tiempos, 95):.2f} max {tiempos.max():.2f}. solved {estados.count('solved')} de 300")
    verificar("MPC resuelve estados aleatorios", estados.count("solved") >= 295)

    itf = InterfazDireccion(cfg["direccion"]["delta_max_rad"], cfg["direccion"]["tasa_max_rad_s"], cfg["ts_s"])
    d1, _, sat_t = itf.aplicar(0.3, 0.0)
    verificar("limite de tasa 1.5 grados por ciclo", abs(d1 - math.radians(1.5)) < 1e-9 and sat_t,
              f"delta={math.degrees(d1):.4f} grados")
    itf2 = InterfazDireccion(0.43, 100.0, 0.05)
    d2, sat_m, _ = itf2.aplicar(1.0, 0.0)
    verificar("limite de magnitud 0.43 rad", abs(d2 - 0.43) < 1e-12 and sat_m)
    out = SalidaSimulada(0.43, 1)
    verificar("delta positivo da eje positivo", out.enviar(0.1, 0.0)[2] > 0)

    print(f"       tamanos fisica base {ctypes.sizeof(SPageFilePhysics)} ampliada {ctypes.sizeof(SPageFilePhysicsAmpliada)}"
          f" graficos {ctypes.sizeof(SPageFileGraphic)} bytes")
    psi, cx, cz, a, Lr = 0.4, 100.0, -50.0, 1.1, 2.633
    ph = SPageFilePhysicsAmpliada()
    gr = SPageFileGraphic()
    ph.heading = psi - math.pi / 2
    ph.speedKmh = 72.0
    gr.carCoordinates[0], gr.carCoordinates[2] = cx, cz
    c, s = math.cos(psi), math.sin(psi)
    rear = (cx - a * c, cz - a * s)
    front = (cx + (Lr - a) * c, cz + (Lr - a) * s)
    lat = (-s * 0.8, c * 0.8)
    for i, (base, lado) in enumerate([(front, 1), (front, -1), (rear, 1), (rear, -1)]):
        ph.tyreContactPoint[i].x = base[0] + lado * lat[0]
        ph.tyreContactPoint[i].z = base[1] + lado * lat[1]
    est = construir_estado(gr, ph, cfg["vehiculo"], 0.0, True)
    verificar("contactos dan batalla y ejes", est.contactos_ok and abs(est.L_medida - Lr) < 1e-4
              and abs(est.x_tras - rear[0]) < 1e-4 and abs(est.z_del - front[1]) < 1e-4,
              f"L={est.L_medida:.4f} motivo={est.motivo_contactos}")
    ph2 = SPageFilePhysicsAmpliada.from_buffer_copy(ph)
    for i, j in ((0, 2), (1, 3)):
        ph2.tyreContactPoint[i], ph2.tyreContactPoint[j] = ph.tyreContactPoint[j], ph.tyreContactPoint[i]
    est2 = construir_estado(gr, ph2, cfg["vehiculo"], 0.0, True)
    verificar("ejes intercambiados se detectan y se usa respaldo", not est2.contactos_ok
              and abs(est2.L_usada - cfg["vehiculo"]["respaldo"]["L_m"]) < 1e-9, est2.motivo_contactos)

    # Seguimiento de vuelta por salto de la posición normalizada. El contador de
    # Assetto Corsa se queda atrás a propósito en esta prueba, porque no cuenta
    # el primer cruce tras la salida.
    sv = SeguidorVuelta(2.0, cfg["vuelta"]["pos_cruce_alta"], cfg["vuelta"]["pos_cruce_baja"])
    posiciones = [0.86, 0.92, 0.98, 0.03, 0.30, 0.70, 0.95, 0.02, 0.20]
    contador = [0, 0, 0, 0, 0, 0, 0, 1, 1]
    fases = [sv.actualizar(k, k * 0.05, contador[k], p, 100, 100, 0.1) for k, p in enumerate(posiciones)]
    verificar("cruce detectado por posicion normalizada",
              fases[2] == "salida" and fases[3] == "medida" and fases[7] == "terminada", str(fases))
    r = sv.resumen(1)
    verificar("resumen de vuelta con comprobacion del contador",
              r["vuelta_completada"] and r["cruces_detectados"] == 2 and r["delta_completedLaps"] == 1
              and abs(r["tiempo_vuelta_s"] - 0.2) < 1e-9, str(r["tiempo_vuelta_s"]))

    with tempfile.TemporaryDirectory() as tmp:
        reg = RegistroCorrida(tmp, "prueba", cfg, RAIZ_P00 / "configs" / "base.json", {"prueba": True})
        reg.agregar({c: 0 for c in COLUMNAS})
        perfil = construir_perfil(monza, cfg["perfil"], "nominal")
        carpeta = reg.escribir(monza, perfil, {"ok": True}, {"carModel": None})
        verificar("registro escribe telemetria perfil y manifiesto",
                  all(os.path.exists(os.path.join(carpeta, n))
                      for n in ("telemetria.csv", "perfil.csv", "manifiesto.json")))

    fallas = [n for n, ok in RESULTADOS if not ok]
    print(f"\n{len(RESULTADOS) - len(fallas)} de {len(RESULTADOS)} verificaciones correctas")
    if fallas:
        print("Fallas:", fallas)
        sys.exit(1)


if __name__ == "__main__":
    main()
