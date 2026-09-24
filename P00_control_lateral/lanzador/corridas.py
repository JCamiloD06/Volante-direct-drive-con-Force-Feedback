"""
Lectura de corridas guardadas para el lanzador.

Revisa las carpetas de corridas, calcula el estado del plan de campaña, arma
el resumen de una corrida y los puntos de verificación de la fase 3. No
modifica ningún archivo de las corridas.

Criterio de estado del plan. Una corrida del plan está pendiente si no tiene
ninguna carpeta con fase campana e id_plan correspondiente. Si tiene al menos
una, cuenta como hecha, haya completado o no la vuelta, porque según la
decisión 5.2 una vuelta fallida es un resultado y no se repite en silencio.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np

from lanzador import plan as mod_plan
from plataforma import procedencia


def cargar_config(ruta):
    with open(ruta, encoding="utf-8") as f:
        return json.load(f)


def directorio_corridas(cfg):
    return procedencia.RAIZ_REPO / cfg["salida"]["directorio_corridas"]


def escanear(directorio):
    directorio = Path(directorio)
    lista = []
    if not directorio.exists():
        return lista
    for ruta in sorted(directorio.glob("*/manifiesto.json")):
        try:
            with open(ruta, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            continue
        meta = data.get("meta", {})
        res = data.get("resumen", {}) or {}
        lista.append({
            "carpeta": ruta.parent,
            "id": data.get("id_corrida"),
            "inicio": data.get("inicio") or "",
            "fase": meta.get("fase"),
            "id_plan": meta.get("id_plan") or "",
            "sesion": meta.get("sesion"),
            "controlador": meta.get("controlador"),
            "perfil": meta.get("perfil"),
            "etiqueta": meta.get("etiqueta") or "",
            "vuelta_completada": bool(res.get("vuelta_completada")),
            "notas": data.get("notas", []),
        })
    return lista


def estado_plan(plan, lista):
    por_id = {}
    for c in lista:
        if c["fase"] == "campana" and c["id_plan"]:
            por_id.setdefault(c["id_plan"], []).append(c)
    filas = []
    siguiente = None
    for r in mod_plan.corridas_en_orden(plan):
        intentos = sorted(por_id.get(r["id_plan"], []), key=lambda c: c["inicio"])
        if not intentos:
            estado = "pendiente"
            if siguiente is None:
                siguiente = r["id_plan"]
        elif any(c["vuelta_completada"] for c in intentos):
            estado = "completada"
        else:
            estado = "vuelta no completada"
        filas.append(dict(r, estado=estado, intentos=len(intentos),
                          carpeta=intentos[-1]["carpeta"].name if intentos else ""))
    return filas, siguiente


def _fmt(v, nd=2, sufijo=""):
    if v is None:
        return "sin dato"
    if isinstance(v, bool):
        return "sí" if v else "no"
    if isinstance(v, (int, float)):
        if isinstance(v, float) and not math.isfinite(v):
            return "sin dato"
        return f"{v:.{nd}f}{sufijo}"
    return str(v)


def leer_manifiesto(carpeta):
    with open(Path(carpeta) / "manifiesto.json", encoding="utf-8") as f:
        return json.load(f)


def resumen_legible(carpeta):
    data = leer_manifiesto(carpeta)
    meta = data.get("meta", {})
    res = data.get("resumen", {}) or {}
    ac = data.get("assetto_corsa", {}) or {}
    cruce = res.get("cruce_inicio") or {}
    lineas = [
        f"Corrida {data.get('id_corrida')}",
        f"Controlador {meta.get('controlador')}, perfil {meta.get('perfil')} f={meta.get('factor_perfil')}, "
        f"fase {meta.get('fase')}, sesión {meta.get('sesion')}, plan {meta.get('id_plan') or 'sin plan'}",
        f"Vehículo {ac.get('carModel')}, pista {ac.get('track')}, AC {ac.get('acVersion')}",
        f"Vuelta completada {_fmt(res.get('vuelta_completada'))}, tiempo de vuelta {_fmt(res.get('tiempo_vuelta_s'), 2, ' s')}",
        f"Ciclos {data.get('ciclos')}, física ampliada {_fmt(meta.get('fisica_ampliada'))}, "
        f"contactos válidos {_fmt(res.get('pct_contactos_ok'), 1, ' %')}, batalla mediana {_fmt(res.get('L_medida_mediana_m'), 3, ' m')}",
        f"Tiempo de cómputo medio {_fmt(res.get('tc_ms_media'))} ms, p95 {_fmt(res.get('tc_ms_p95'))} ms, "
        f"máximo {_fmt(res.get('tc_ms_max'))} ms, ciclos sobre 50 ms {_fmt(res.get('pct_tc_mayor_50ms'), 2, ' %')}",
        f"Periodo medio {_fmt(res.get('periodo_ms_media'))} ms, retardo p95 {_fmt(res.get('retardo_ms_p95'))} ms",
        f"Saturación de magnitud {_fmt(res.get('pct_sat_magnitud'), 2, ' %')}, de tasa {_fmt(res.get('pct_sat_tasa'), 2, ' %')}, "
        f"respaldo del controlador {_fmt(res.get('pct_respaldo_controlador'), 2, ' %')}",
        f"Primer cruce de meta, diferencia con el objetivo {_fmt(cruce.get('diferencia_velocidad_kmh'), 1, ' km/h')}, "
        f"máximo |e_y| previo {_fmt(cruce.get('max_abs_e_y_ventana_m'), 2, ' m')}",
        f"Notas {'; '.join(data.get('notas', [])) or 'ninguna'}",
        f"Carpeta {carpeta}",
    ]
    return "\n".join(lineas)


def _columnas(carpeta, nombres):
    ruta = Path(carpeta) / "telemetria.csv"
    datos = {n: [] for n in nombres}
    fases = []
    with open(ruta, newline="", encoding="utf-8") as f:
        for fila in csv.DictReader(f):
            fases.append(fila.get("fase_vuelta", ""))
            for n in nombres:
                v = fila.get(n, "")
                if v in ("True", "False"):
                    datos[n].append(1.0 if v == "True" else 0.0)
                else:
                    try:
                        datos[n].append(float(v))
                    except (TypeError, ValueError):
                        datos[n].append(np.nan)
    return {n: np.array(v) for n, v in datos.items()}, np.array(fases)


def verificacion_fase3(carpeta, cfg):
    """
    Puntos de la fase 3. Cada punto devuelve nombre, valor, criterio y si
    cumple. cumple es None cuando el punto es informativo y no hay un umbral
    decidido.
    """
    data = leer_manifiesto(carpeta)
    meta = data.get("meta", {})
    res = data.get("resumen", {}) or {}
    ac = data.get("assetto_corsa", {}) or {}
    cols, fases = _columnas(carpeta, [
        "L_medida_m", "desalineacion_ejes_rad", "deriva_rad", "v_kmh", "tc_ms", "ruedas_fuera",
        "steer_norm", "steer_angle_ac", "tasa_guinada_local", "acc_g_y", "L_usada_m", "periodo_ms"])
    medida = fases == "medida"
    usar = medida if medida.any() else np.ones(len(fases), dtype=bool)
    items = []

    def item(nombre, valor, criterio, cumple):
        items.append({"nombre": nombre, "valor": valor, "criterio": criterio, "cumple": cumple})

    esperado = cfg["vehiculo"]["modelo_esperado"]
    item("Vehículo", ac.get("carModel"), f"igual a {esperado}", ac.get("carModel") == esperado)
    item("Física ampliada disponible", _fmt(meta.get("fisica_ampliada")),
         "sí, necesaria para medir ejes con contactos, I2", bool(meta.get("fisica_ampliada")))
    item("Ciclos con contactos válidos", _fmt(res.get("pct_contactos_ok"), 1, " %"),
         "informativo, se espera cercano a 100 %", None)

    L = cols["L_medida_m"][np.isfinite(cols["L_medida_m"])]
    if L.size:
        item("Batalla medida", f"mediana {np.median(L):.3f} m, p5 {np.percentile(L, 5):.3f}, p95 {np.percentile(L, 95):.3f}",
             "estable entre ciclos", None)
    else:
        item("Batalla medida", "sin ciclos con contactos válidos", "estable entre ciclos", False)
    des = np.degrees(np.abs(cols["desalineacion_ejes_rad"]))
    des = des[np.isfinite(des)]
    item("Desalineación entre ejes y ψ", f"mediana {np.median(des):.2f} grados" if des.size else "sin dato",
         "pequeña, confirma el desfase de 90 grados de I1", None)

    m_v = usar & (cols["v_kmh"] > 30)
    der = np.degrees(np.abs(cols["deriva_rad"][m_v]))
    der = der[np.isfinite(der)]
    item("Deriva entre rumbo y ψ", f"mediana {np.median(der):.2f}, p95 {np.percentile(der, 95):.2f} grados" if der.size else "sin dato",
         "comparar con 0.21 y 1.36 grados de corridas previas", None)

    item("Vuelta medida completada", _fmt(res.get("vuelta_completada")), "sí", bool(res.get("vuelta_completada")))
    cruce = res.get("cruce_inicio") or {}
    tol = cfg["vuelta"]["tolerancia_velocidad_cruce_kmh"]
    dif = cruce.get("diferencia_velocidad_kmh")
    item("Velocidad al primer cruce", _fmt(dif, 1, " km/h"), f"|diferencia| menor a {tol} km/h, I7",
         None if dif is None else abs(dif) < tol)
    item("Error lateral antes del primer cruce", _fmt(cruce.get("max_abs_e_y_ventana_m"), 2, " m"),
         "estabilizado, I7", None)

    pct50 = res.get("pct_tc_mayor_50ms")
    item("Ciclos con tiempo de cómputo bajo 50 ms", _fmt(None if pct50 is None else 100.0 - pct50, 2, " %"),
         "al menos 99 %, decisión 1.4", None if pct50 is None else (100.0 - pct50) >= 99.0)
    item("Periodo medio", _fmt(res.get("periodo_ms_media"), 2, " ms"), "cercano a 50 ms", None)
    item("Saturaciones de magnitud y tasa",
         f"{_fmt(res.get('pct_sat_magnitud'), 2, ' %')} y {_fmt(res.get('pct_sat_tasa'), 2, ' %')}",
         "informativo, I5", None)
    item("Respaldo del controlador", _fmt(res.get("pct_respaldo_controlador"), 2, " %"), "informativo, I4.4", None)
    fuera = cols["ruedas_fuera"][medida] > 2
    eventos = int(np.sum(fuera[1:] & ~fuera[:-1]) + (1 if fuera.size and fuera[0] else 0)) if fuera.size else 0
    item("Salidas de pista en la vuelta medida", str(eventos), "informativo, decisión 5.1", None)

    m_s = np.abs(cols["steer_norm"]) > 0.01
    if m_s.sum() > 10:
        corr = float(np.corrcoef(cols["steer_norm"][m_s], cols["steer_angle_ac"][m_s])[0, 1])
        item("Signo entre comando vJoy y steerAngle", f"correlación {corr:.3f}",
             "positiva, fue 0.999 en corridas previas", corr > 0)
    else:
        item("Signo entre comando vJoy y steerAngle", "pocos ciclos con dirección", "positiva", None)

    # Reidentificación de la cadena de dirección con la batalla medida.
    # Ángulo cinemático δ = atan(L r / v) con r la tasa de guiñada local, en
    # ciclos de baja aceleración lateral. La pendiente frente al comando
    # normalizado es rueda_rad_eje_completo. El signo del eje de guiñada
    # local no está verificado, por eso se usa el valor absoluto de la pendiente.
    v_ms = cols["v_kmh"] / 3.6
    m_r = (usar & (cols["v_kmh"] > 30) & (np.abs(cols["acc_g_y"]) < 0.25) & (np.abs(cols["steer_norm"]) > 0.02)
           & np.isfinite(cols["tasa_guinada_local"]) & np.isfinite(cols["L_usada_m"]))
    config_rueda = cfg["direccion"]["rueda_rad_eje_completo"]
    if m_r.sum() > 50:
        delta_kin = np.arctan(cols["L_usada_m"][m_r] * cols["tasa_guinada_local"][m_r] / v_ms[m_r])
        x = cols["steer_norm"][m_r]
        pendiente = float(np.sum(x * delta_kin) / np.sum(x * x))
        r2 = 1.0 - float(np.sum((delta_kin - pendiente * x) ** 2) / np.sum((delta_kin - delta_kin.mean()) ** 2))
        diferencia = abs(abs(pendiente) - config_rueda) / config_rueda * 100.0
        item("Constante de la cadena de dirección",
             f"{abs(pendiente):.4f} rad, R² {r2:.3f}, signo de la pendiente {'+' if pendiente > 0 else '-'}, "
             f"{m_r.sum()} ciclos, diferencia {diferencia:.1f} % con {config_rueda}",
             "recalibrar si difiere más de 15 %, criterio de DECISIONS", diferencia <= 15.0)
    else:
        item("Constante de la cadena de dirección", "pocos ciclos útiles o sin tasa de guiñada local",
             "requiere física ampliada y tramos con dirección", None)
    return items
