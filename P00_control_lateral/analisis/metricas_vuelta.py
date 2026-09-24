"""
Métricas por vuelta y por región de curvatura, Tabla 3 del manuscrito y decisión 1.

Solo usa los ciclos de la vuelta medida, fase medida, decisión I7. La región
de cada ciclo sale de la curvatura en la proyección de la posición de Assetto
Corsa. El error lateral se calcula en esa posición y en el eje trasero, porque
el punto de la métrica primaria está PENDIENTE de decisión.

Escribe metricas.json dentro de la carpeta de cada corrida.

Uso desde la raíz del repositorio.
    python P00_control_lateral/analisis/metricas_vuelta.py data/raw/p00/corridas/<id_corrida>
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

RAIZ_P00 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ_P00))

from plataforma.regiones import asignar_region  # noqa: E402


def _num(filas, col):
    salida = []
    for f in filas:
        v = f.get(col, "")
        try:
            salida.append(float(v))
        except (TypeError, ValueError):
            salida.append(np.nan)
    return np.array(salida)


def _bool(filas, col):
    return np.array([str(f.get(col, "")).strip() in ("True", "true", "1") for f in filas])


def _rmse(a):
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a ** 2))) if a.size else None


def _mae(a):
    a = a[np.isfinite(a)]
    return float(np.mean(np.abs(a))) if a.size else None


def _max_abs(a):
    a = a[np.isfinite(a)]
    return float(np.max(np.abs(a))) if a.size else None


def _pct(m):
    return float(np.mean(m) * 100.0) if m.size else None


def metricas_grupo(d, m):
    tc = d["tc_ms"][m]
    fuera = d["ruedas_fuera"][m] > 2
    eventos = int(np.sum(fuera[1:] & ~fuera[:-1]) + (1 if fuera.size and fuera[0] else 0))
    tasa = d["tasa_delta"][m]
    tasa = tasa[np.isfinite(tasa)]
    salto = d["salto_delta"][m]
    return {
        "ciclos": int(m.sum()),
        "rmse_e_y_cg_m": _rmse(d["e_y_cg"][m]),
        "mae_e_y_cg_m": _mae(d["e_y_cg"][m]),
        "max_abs_e_y_cg_m": _max_abs(d["e_y_cg"][m]),
        "rmse_e_y_tras_m": _rmse(d["e_y_tras"][m]),
        "mae_e_y_tras_m": _mae(d["e_y_tras"][m]),
        "max_abs_e_y_tras_m": _max_abs(d["e_y_tras"][m]),
        "rmse_e_psi_cg_rad": _rmse(d["e_psi_cg"][m]),
        "rms_tasa_delta_rad_s": float(np.sqrt(np.mean(tasa ** 2))) if tasa.size else None,
        "variacion_total_delta_rad": float(np.nansum(np.abs(salto))),
        "pico_abs_delta_rad": _max_abs(d["delta"][m]),
        "rmse_v_kmh": _rmse(d["e_v"][m]),
        "mae_v_kmh": _mae(d["e_v"][m]),
        "max_abs_e_v_kmh": _max_abs(d["e_v"][m]),
        "tc_ms_media": float(np.nanmean(tc)) if tc.size else None,
        "tc_ms_p95": float(np.nanpercentile(tc, 95)) if tc.size else None,
        "tc_ms_max": float(np.nanmax(tc)) if tc.size else None,
        "pct_tc_mayor_50ms": _pct(tc > 50.0),
        "pct_sat_magnitud": _pct(d["sat_mag"][m]),
        "pct_sat_tasa": _pct(d["sat_tasa"][m]),
        "pct_respaldo_controlador": _pct(d["respaldo"][m] > 0),
        "pct_direccion_retenida": _pct(d["retenida"][m]),
        "pct_ciclos_mas_de_dos_ruedas_fuera": _pct(fuera),
        "eventos_salida_pista": eventos,
        "pct_contactos_ok": _pct(d["contactos_ok"][m]),
    }


def procesar(carpeta, cfg):
    carpeta = Path(carpeta)
    with open(carpeta / "telemetria.csv", newline="", encoding="utf-8") as f:
        filas = [r for r in csv.DictReader(f) if r.get("fase_vuelta") == "medida"]
    manifiesto = json.load(open(carpeta / "manifiesto.json", encoding="utf-8"))
    resultado = {
        "id_corrida": manifiesto.get("id_corrida"),
        "controlador": manifiesto.get("meta", {}).get("controlador"),
        "perfil": manifiesto.get("meta", {}).get("perfil"),
        "sesion": manifiesto.get("meta", {}).get("sesion"),
        "vuelta_completada": manifiesto.get("resumen", {}).get("vuelta_completada"),
        "tiempo_vuelta_s": manifiesto.get("resumen", {}).get("tiempo_vuelta_s"),
        "cruce_inicio": manifiesto.get("resumen", {}).get("cruce_inicio"),
        "punto_metrica_primaria": cfg["metricas"]["punto_error_lateral"],
        "grupos": {},
    }
    if not filas:
        resultado["aviso"] = "sin ciclos en fase medida"
    else:
        t = _num(filas, "t_s")
        delta = _num(filas, "delta_aplicado_rad")
        salto = np.concatenate([[np.nan], np.diff(delta)])
        dt = np.concatenate([[np.nan], np.diff(t)])
        d = {
            "e_y_cg": _num(filas, "e_y_cg_m"),
            "e_y_tras": _num(filas, "e_y_tras_m"),
            "e_psi_cg": _num(filas, "e_psi_cg_rad"),
            "delta": delta,
            "salto_delta": salto,
            "tasa_delta": np.where(dt > 1e-4, salto / dt, np.nan),
            "e_v": _num(filas, "v_kmh") - _num(filas, "v_ref_kmh"),
            "tc_ms": _num(filas, "tc_ms"),
            "sat_mag": _bool(filas, "sat_magnitud"),
            "sat_tasa": _bool(filas, "sat_tasa"),
            "respaldo": _num(filas, "ctrl_respaldo"),
            "retenida": _bool(filas, "direccion_retenida"),
            "ruedas_fuera": _num(filas, "ruedas_fuera"),
            "contactos_ok": _bool(filas, "contactos_ok"),
        }
        reg = cfg["regiones"]
        regiones = asignar_region(_num(filas, "kappa_cg"), reg["radio_baja_m"], reg["radio_alta_m"])
        resultado["grupos"]["vuelta"] = metricas_grupo(d, np.ones(len(filas), dtype=bool))
        for nombre in ("baja", "media", "alta"):
            resultado["grupos"][nombre] = metricas_grupo(d, regiones == nombre)
    with open(carpeta / "metricas.json", "w", encoding="utf-8") as f:
        json.dump(resultado, f, indent=2, ensure_ascii=False)
    return resultado


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("corridas", nargs="+")
    ap.add_argument("--config", default=str(RAIZ_P00 / "configs" / "base.json"))
    args = ap.parse_args()
    with open(args.config, encoding="utf-8") as f:
        cfg = json.load(f)
    for carpeta in args.corridas:
        r = procesar(carpeta, cfg)
        print(f"{r['id_corrida']}  vuelta completada {r['vuelta_completada']}")
        for nombre, g in r["grupos"].items():
            print(f"  {nombre:6s} ciclos {g['ciclos']:5d}  RMSE e_y cg {g['rmse_e_y_cg_m']}  "
                  f"tc p95 {g['tc_ms_p95']}  pct tc>50 {g['pct_tc_mayor_50ms']}")


if __name__ == "__main__":
    main()
